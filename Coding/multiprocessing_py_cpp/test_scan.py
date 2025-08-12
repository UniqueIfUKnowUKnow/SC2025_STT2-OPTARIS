# =================================================================
# Python Scanner with Distance Calibration and Initial Scan
#
# Purpose: This script first performs a calibration phase to find an
# average background distance, then enters a scanning phase to
# detect any object that is significantly closer.
# =================================================================

# --- Standard Library Imports ---
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import pigpio            # For stable, hardware-based PWM for the servo
import time              # For creating delays
import serial            # For reading data from the LiDAR's serial port
import threading         # To run the LiDAR reader in the background
import queue             # For thread-safe data sharing between threads
import statistics        # For easily calculating the average distance

# --- Pin & Port Configuration (BCM numbering) ---
# Stepper Motor Pins
DIR_PIN    = 26
STEP_PIN   = 13
ENABLE_PIN = 23
RESET_PIN  = 19
M0_PIN     = 0
M1_PIN     = 5
M2_PIN     = 6

# Servo Motor Pin
SERVO_PIN  = 2

# LiDAR Serial Port (GPIO 14/15 -> /dev/ttyS0)
LIDAR_SERIAL_PORT = '/dev/ttyS0'
LIDAR_BAUD_RATE = 115200

# --- Motor & Movement Settings ---
STEPS_PER_REVOLUTION = 6400
STEPPER_PULSE_DELAY = 0.0001
MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500

# --- Stepper Sweep Settings ---
SWEEP_RANGE_DEGREES = 40
STEPS_FOR_SWEEP = int((SWEEP_RANGE_DEGREES / 360.0) * STEPS_PER_REVOLUTION)

# --- Calibration & Detection Settings ---
CALIBRATION_SWEEPS = 2
# An object is "significant" if it's 20% closer than the average.
DETECTION_THRESHOLD_FACTOR = 0.8

# --- LiDAR Reader Thread ---
class LidarReader(threading.Thread):
    """
    A dedicated thread that continuously reads data from the TFmini-S LiDAR.
    """
    def __init__(self, port, baudrate, data_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.data_queue = data_queue
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.ser.flushInput()
        self.frame_header = 0x59
        print("LiDAR Reader thread initialized.")

    def run(self):
        while True:
            if self.ser.read(1) == b'\x59':
                if self.ser.read(1) == b'\x59':
                    frame = b'\x59\x59' + self.ser.read(7)
                    if len(frame) == 9:
                        checksum = sum(frame[:-1]) & 0xFF
                        if checksum == frame[8]:
                            distance_cm = frame[2] + (frame[3] << 8)
                            if distance_cm > 0:
                                self.data_queue.put(distance_cm)

# --- Setup and Control Functions ---
def setup_stepper_gpio():
    """
    Configures the RPi's GPIO pins for controlling the DRV8825 stepper driver.
    """
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pins = [DIR_PIN, STEP_PIN, ENABLE_PIN, RESET_PIN, M0_PIN, M1_PIN, M2_PIN]
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)
    GPIO.output(ENABLE_PIN, GPIO.LOW)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    GPIO.output(M0_PIN, GPIO.HIGH)
    GPIO.output(M1_PIN, GPIO.HIGH)
    GPIO.output(M2_PIN, GPIO.HIGH)

def set_servo_angle(pi, angle):
    """
    Calculates the required pulse width and sets the servo to a specific angle.
    """
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

    stepper_steps_taken = 0
    stepper_direction_cw = True
    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    constant_servo_angle = 20
    set_servo_angle(pi, constant_servo_angle)
    print(f"Servo angle set to {constant_servo_angle} degrees.")

    calibration_distances = []
    sweeps_completed = 0
    average_distance = 0

    try:
        # =================================================================
        # 1. CALIBRATION PHASE
        # =================================================================
        while current_state == "CALIBRATING":
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
            stepper_steps_taken += 1

            try:
                distance = lidar_data_queue.get_nowait()
                calibration_distances.append(distance)
            except queue.Empty:
                pass

            if stepper_steps_taken >= STEPS_FOR_SWEEP:
                stepper_steps_taken = 0
                stepper_direction_cw = not stepper_direction_cw
                GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)
                sweeps_completed += 0.5
                print(f"Calibration sweep {int(sweeps_completed * 2)} of {CALIBRATION_SWEEPS * 2} halfs completed...")

                if sweeps_completed >= CALIBRATION_SWEEPS:
                    if calibration_distances:
                        average_distance = statistics.mean(calibration_distances)
                        print("\n" + "="*30)
                        print("CALIBRATION COMPLETE")
                        print(f"Average Background Distance: {average_distance:.2f} cm")
                        print("="*30 + "\n")
                        current_state = states[1] # Transition to the next state
                        print(f"Current State: {current_state}")
                    else:
                        print("Calibration Failed: No LiDAR data collected.")
                        current_state = "FINISHED"
        
        # Reset stepper state for the next phase
        stepper_steps_taken = 0
        
        # =================================================================
        # 2. INITIAL SCAN PHASE
        # =================================================================
        while current_state == "SCANNING":
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
            stepper_steps_taken += 1

            # --- Detection Logic ---
            try:
                distance = lidar_data_queue.get_nowait()
                
                # Compare current distance to the calibrated average
                if distance < (average_distance * DETECTION_THRESHOLD_FACTOR):
                    # Calculate the stepper's current angle within the sweep
                    # Note: This is a simplified angle relative to the sweep start.
                    # 160 is the starting angle of the 40-degree sweep (180 - 40/2).
                    angle_offset = (stepper_steps_taken / STEPS_FOR_SWEEP) * SWEEP_RANGE_DEGREES
                    current_stepper_angle = 160 + angle_offset if stepper_direction_cw else 200 - angle_offset

                    print("\n" + "="*40)
                    print(f"TARGET DETECTED!")
                    print(f"  -> Distance: {distance} cm (Average was {average_distance:.2f} cm)")
                    print(f"  -> Stepper Angle: {current_stepper_angle:.1f}°")
                    print(f"  -> Servo Angle: {constant_servo_angle}°")
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