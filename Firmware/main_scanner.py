# --- Standard Library Imports ---
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import pigpio            # For stable, hardware-based PWM for the servo
import time              # For creating delays
import serial            # For reading data from the LiDAR's serial port
import threading         # To run the LiDAR reader in the background
import queue             # For thread-safe data sharing between threads
import statistics        # For easily calculating the average distance
from lidar_reader import LidarReader
from constants import *
from stepper_setup import setup_stepper_gpio

# --- Setup and Control Functions ---

def set_servo_angle(pi, angle):
    """Calculates the required pulse width and sets the servo to a specific angle."""
    pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

# --- Main Application ---
def main():
    """
    The main entry point of the program. It handles setup, state management,
    and the main control loop for the calibration and scanning process.
    """
    # --- Setup ---
    setup_stepper_gpio()
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Is it running?")
        return

    lidar_data_queue = queue.Queue()
    lidar_thread = LidarReader(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, lidar_data_queue)
    lidar_thread.start()

    # --- State Machine and Variables ---
    states = ["CALIBRATING", "SCANNING", "DETECTED"]
    current_state = states[0]
    print(f"Current State: {current_state}")

    # Motor state variables
    stepper_steps_taken = 0
    stepper_direction_cw = True
    GPIO.output(DIR_PIN, GPIO.HIGH)
    servo_angle = SERVO_SWEEP_START
    servo_direction_up = True
    set_servo_angle(pi, servo_angle)
    last_servo_update = time.time()
    
    # Data collection variables
    calibration_distances = []
    sweeps_completed = 0
    average_distance = 0
    
    # --- FIX: New variable to track consecutive close readings. ---
    consecutive_detections = 0

    try:
        # =================================================================
        # 1. CALIBRATION PHASE
        # =================================================================
        while current_state == "CALIBRATING":
            # --- Perform one complete 180-degree stepper sweep ---
            while stepper_steps_taken < STEPS_FOR_SWEEP:
                GPIO.output(STEP_PIN, GPIO.HIGH)
                time.sleep(STEPPER_PULSE_DELAY)
                GPIO.output(STEP_PIN, GPIO.LOW)
                time.sleep(STEPPER_PULSE_DELAY)
                stepper_steps_taken += 1
                
                # --- Collect LiDAR Data during sweep ---
                try:
                    distance = lidar_data_queue.get_nowait()
                    if distance > 0:  # Only store valid readings
                        # Calculate current stepper angle
                        angle_offset = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                        current_stepper_angle = angle_offset if stepper_direction_cw else STEPPER_SWEEP_DEGREES - angle_offset
                        
                        # Store calibration data with positions
                        calibration_data_point = {
                            'distance': distance,
                            'stepper_angle': current_stepper_angle,
                            'servo_angle': servo_angle,
                            'stepper_direction_cw': stepper_direction_cw
                        }
                        calibration_distances.append(calibration_data_point)
                except queue.Empty:
                    pass
            
            # --- End of sweep reached ---
            stepper_steps_taken = 0
            sweeps_completed += 1
            print(f"Calibration sweep {sweeps_completed} completed at servo angle {servo_angle}°")
            
            # --- Move servo up by 2 degrees for next sweep ---
            servo_angle += 2
            set_servo_angle(pi, servo_angle)
            time.sleep(0.1)  # Brief pause for servo to settle
            
            # --- Reverse stepper direction for next sweep ---
            stepper_direction_cw = not stepper_direction_cw
            GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)
            
            # --- Check if calibration is complete ---
            if servo_angle > SERVO_SWEEP_END:
                if calibration_distances:
                    # Calculate average distance from all collected data
                    all_distances = [point['distance'] for point in calibration_distances]
                    average_distance = statistics.mean(all_distances)
                    
                    print("\n" + "="*40)
                    print("CALIBRATION COMPLETE")
                    print(f"Total data points collected: {len(calibration_distances)}")
                    print(f"Average Background Distance: {average_distance:.2f} cm")
                    print(f"Servo range covered: {SERVO_SWEEP_START}° to {servo_angle-2}°")
                    print("="*40 + "\n")
                    
                    current_state = states[1]
                    print(f"Current State: {current_state}")
                else:
                    print("Calibration Failed: No LiDAR data collected.")
                    current_state = "FINISHED"
        
        # =================================================================
        # 2. INITIAL SCAN PHASE
        # =================================================================
        while current_state == "SCANNING":
            # --- Interleaved Motor Control (same as calibration) ---
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
            stepper_steps_taken += 1

            if time.time() - last_servo_update > SERVO_UPDATE_INTERVAL:
                last_servo_update = time.time()
                if servo_direction_up:
                    servo_angle += 1
                    if servo_angle >= SERVO_SWEEP_END: servo_direction_up = False
                else:
                    servo_angle -= 1
                    if servo_angle <= SERVO_SWEEP_START: servo_direction_up = True
                set_servo_angle(pi, servo_angle)

            # --- Detection Logic ---
            try:
                distance = lidar_data_queue.get_nowait()
                
                # --- FIX: Check for consecutive detections ---
                if distance < (average_distance * DETECTION_THRESHOLD_FACTOR):
                    # If the reading is close, increment our confidence counter.
                    consecutive_detections += 1
                else:
                    # If we get a reading that is far away, reset the counter.
                    consecutive_detections = 0

                # --- FIX: Only confirm a target if our confidence is high enough. ---
                if consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                    # We have seen enough consecutive close readings to be sure.
                    angle_offset = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                    current_stepper_angle = angle_offset if stepper_direction_cw else STEPPER_SWEEP_DEGREES - angle_offset

                    print("\n" + "="*40)
                    print(f"TARGET DETECTED!")
                    print(f"  -> Distance: {distance} cm (Average was {average_distance:.2f} cm)")
                    print(f"  -> Stepper Angle: {current_stepper_angle:.1f}°")
                    print(f"  -> Servo Angle: {servo_angle}°")
                    print("="*40 + "\n")
                    current_state = states[2] # Change state to stop the scan
                    
            except queue.Empty:
                pass

            # Reverse direction at the end of the sweep
            if stepper_steps_taken >= STEPS_FOR_SWEEP:
                stepper_steps_taken = 0
                stepper_direction_cw = not stepper_direction_cw
                GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)

        print("Scanning complete.")

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Cleaning up...")
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()

