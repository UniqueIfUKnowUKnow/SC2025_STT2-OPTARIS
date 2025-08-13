# =================================================================
# Python Scanner with Re-Detection Loop
#
# Purpose: This script calibrates the background, then enters a
# continuous scanning loop. After each detection, it pauses and
# then automatically resumes scanning until 3 targets are found.
# =================================================================

# --- Standard Library Imports ---
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import pigpio            # For stable, hardware-based PWM for the servo
import time              # For creating delays
import serial            # For reading data from the LiDAR's serial port
import threading         # To run the LiDAR reader in the background
import queue             # For thread-safe data sharing between threads
import statistics        # For easily calculating the average distance
import keyboard          # For handling keyboard interrupts

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
# --- CHANGE: Added back servo update interval for sweeping motion ---
SERVO_UPDATE_INTERVAL = 0.01

# --- Stepper and Servo Sweep Settings ---
STEPPER_SWEEP_DEGREES = 40
STEPS_FOR_SWEEP = int((STEPPER_SWEEP_DEGREES / 360.0) * STEPS_PER_REVOLUTION)
# --- CHANGE: New settings for servo sweep range ---
SERVO_SWEEP_START = 5
SERVO_SWEEP_END = 25


# --- Calibration & Detection Settings ---
CALIBRATION_SWEEPS = 2
DETECTION_THRESHOLD_FACTOR = 0.8
DETECTION_CONFIDENCE_THRESHOLD = 5
TOTAL_DETECTIONS_TO_FIND = 3


class TFminiS:
    """
    A class to manage communication with the TFmini-S LiDAR sensor.
    """
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.frame_length = 9
        self.frame_header = 0x59

    def open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.flushInput()
            if self.ser.isOpen():
                print(f"Successfully opened serial port: {self.port} at {self.baudrate} baud.")
                return True
            else:
                print(f"Failed to open serial port: {self.port}.")
                return False
        except serial.SerialException as e:
            print(f"Error opening serial port {self.port}: {e}")
            return False

    def close_serial(self):
        if self.ser and self.ser.isOpen():
            self.ser.close()
            print("Serial port closed.")

    def read_data(self):
        """
        Reads a 9-byte data frame from the sensor, validates it, and parses it.
        Returns distance in cm or None if data is invalid.
        """
        while True:
            if self.ser.in_waiting >= self.frame_length:
                byte1 = self.ser.read(1)
                if not byte1:
                    continue

                if ord(byte1) == self.frame_header:
                    byte2 = self.ser.read(1)
                    if not byte2:
                        continue

                    if ord(byte2) == self.frame_header:
                        remaining_bytes = self.ser.read(self.frame_length - 2)
                        frame = byte1 + byte2 + remaining_bytes
                        checksum = frame[-1]
                        calculated_checksum = sum(frame[:-1]) & 0xFF

                        if checksum == calculated_checksum:
                            distance = frame[2] + (frame[3] << 8)
                            if distance != 65535:  # Invalid reading check
                                return distance
                        else:
                            self.ser.flushInput()
                            return None
            else:
                time.sleep(0.001)


# --- LiDAR Reader Thread ---
class LidarReader(threading.Thread):
    """A dedicated thread that continuously reads data from the TFmini-S LiDAR."""
    def __init__(self, port, baudrate, data_queue):
        super().__init__(daemon=True)
        self.data_queue = data_queue
        self.lidar = TFminiS(port, baudrate)
        if not self.lidar.open_serial():
            raise RuntimeError("Failed to initialize LiDAR sensor")
        print("LiDAR Reader thread initialized.")

    def run(self):
        while True:
            distance = self.lidar.read_data()
            if distance is not None:
                self.data_queue.put(distance)

    def cleanup(self):
        """Clean up resources when done"""
        self.lidar.close_serial()


# --- Setup and Control Functions ---
def setup_stepper_gpio():
    """Configures the RPi's GPIO pins for controlling the DRV8825 stepper driver."""
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
    states = ["CALIBRATING", "SCANNING", "FINISHED"]
    current_state = states[0]
    print(f"Current State: {current_state}")

    stepper_steps_taken = 0
    stepper_direction_cw = True
    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    # --- CHANGE: Initialize servo for sweeping motion ---
    servo_angle = SERVO_SWEEP_START
    servo_direction_up = True
    set_servo_angle(pi, servo_angle)
    last_servo_update = time.time()
    print(f"Servo sweep range set to {SERVO_SWEEP_START}° - {SERVO_SWEEP_END}°.")


    calibration_distances = []
    sweeps_completed = 0
    average_distance = 0
    
    # Counter for the detection loop.
    detections_found = 0
    consecutive_detections = 0

    try:
        # =================================================================
        # 1. CALIBRATION PHASE
        # =================================================================
        while current_state == "CALIBRATING":
            # --- Interleaved Motor Control ---
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
            stepper_steps_taken += 1

            # --- CHANGE: Added servo sweep logic to calibration loop ---
            if time.time() - last_servo_update > SERVO_UPDATE_INTERVAL:
                last_servo_update = time.time()
                if servo_direction_up:
                    servo_angle += 1
                    if servo_angle >= SERVO_SWEEP_END: servo_direction_up = False
                else:
                    servo_angle -= 1
                    if servo_angle <= SERVO_SWEEP_START: servo_direction_up = True
                set_servo_angle(pi, servo_angle)

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
                        current_state = states[1]
                        print(f"Current State: {current_state}")
                    else:
                        print("Calibration Failed: No LiDAR data collected.")
                        current_state = "FINISHED"
        
        # Reset stepper state for the next phase
        stepper_steps_taken = 0
        
        # Clear the LiDAR data queue before starting the scan.
        with lidar_data_queue.mutex:
            lidar_data_queue.queue.clear()
        
        # =================================================================
        # 2. INITIAL SCAN & RE-DETECTION PHASE
        # =================================================================
        # The main loop now continues until 3 detections are found.
        while detections_found < TOTAL_DETECTIONS_TO_FIND:
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
            stepper_steps_taken += 1

            # --- CHANGE: Added servo sweep logic to detection loop ---
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
                
                if distance < (average_distance * DETECTION_THRESHOLD_FACTOR):
                    consecutive_detections += 1
                else:
                    consecutive_detections = 0

                if consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                    center_angle = 180
                    half_sweep = STEPPER_SWEEP_DEGREES / 2.0
                    angle_offset = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                    
                    if stepper_direction_cw:
                        current_stepper_angle = (center_angle - half_sweep) + angle_offset
                    else:
                        current_stepper_angle = (center_angle + half_sweep) - angle_offset

                    detections_found += 1
                    print("\n" + "="*40)
                    print(f"TARGET DETECTED! ({detections_found}/{TOTAL_DETECTIONS_TO_FIND})")
                    print(f"  -> Distance: {distance} cm (Average was {average_distance:.2f} cm)")
                    print(f"  -> Stepper Angle: {current_stepper_angle:.1f}°")
                    # --- CHANGE: Use the current servo angle variable ---
                    print(f"  -> Servo Angle: {servo_angle}°")
                    print("="*40)
                    
                    # Pause and prepare for the next scan.
                    if detections_found < TOTAL_DETECTIONS_TO_FIND:
                        print("Pausing for 1 second, then resuming scan...")
                        time.sleep(1)
                        # Clear the queue and confidence to avoid instant re-detection
                        with lidar_data_queue.mutex:
                            lidar_data_queue.queue.clear()
                        consecutive_detections = 0
                    
            except queue.Empty:
                pass

            # Reverse direction at the end of the sweep
            if stepper_steps_taken >= STEPS_FOR_SWEEP:
                stepper_steps_taken = 0
                stepper_direction_cw = not stepper_direction_cw
                GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)
                consecutive_detections = 0

        print("\nAll 3 targets detected. Scanning complete.")


        # =================================================================
        # 3. ASSURANCE SCAN
        # =================================================================
        print("\nStarting Assurance Scan phase...")
        
        # Reset detection counters for assurance scan
        detections_verified = 0
        detection_points = []
        
        # Modify sweep parameters for zig-zag pattern
        ASSURANCE_STEPPER_SWEEP = 20  # Reduced sweep angle for finer scanning
        ASSURANCE_STEPS = int((ASSURANCE_STEPPER_SWEEP / 360.0) * STEPS_PER_REVOLUTION)
        
        # Reset movement variables
        stepper_steps_taken = 0
        consecutive_detections = 0
        
        # Clear the LiDAR queue before starting assurance scan
        with lidar_data_queue.mutex:
            lidar_data_queue.queue.clear()
            
        while detections_verified < TOTAL_DETECTIONS_TO_FIND:
            # Stepper motor control with shorter sweeps
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
            stepper_steps_taken += 1
            
            # Faster servo sweeping for zig-zag pattern
            if time.time() - last_servo_update > (SERVO_UPDATE_INTERVAL / 2):
                last_servo_update = time.time()
                if servo_direction_up:
                    servo_angle += 2  # Faster vertical movement
                    if servo_angle >= SERVO_SWEEP_END:
                        servo_direction_up = False
                else:
                    servo_angle -= 2
                    if servo_angle <= SERVO_SWEEP_START:
                        servo_direction_up = True
                set_servo_angle(pi, servo_angle)
            
            try:
                distance = lidar_data_queue.get_nowait()
                
                if distance < (average_distance * DETECTION_THRESHOLD_FACTOR):
                    consecutive_detections += 1
                else:
                    consecutive_detections = 0
                    
                if consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                    # Calculate current angles
                    center_angle = 180
                    half_sweep = ASSURANCE_STEPPER_SWEEP / 2.0
                    angle_offset = (stepper_steps_taken / ASSURANCE_STEPS) * ASSURANCE_STEPPER_SWEEP
                    
                    current_stepper_angle = (center_angle - half_sweep) + angle_offset if stepper_direction_cw else (center_angle + half_sweep) - angle_offset
                    
                    # Check if this is a new detection point (not too close to existing ones)
                    is_new_point = True
                    for point in detection_points:
                        if (abs(point['stepper'] - current_stepper_angle) < 5 and 
                            abs(point['servo'] - servo_angle) < 5):
                            is_new_point = False
                            break
                    
                    if is_new_point:
                        detection_points.append({
                            'stepper': current_stepper_angle,
                            'servo': servo_angle,
                            'distance': distance
                        })
                        detections_verified += 1
                        print("\n" + "="*40)
                        print(f"Target Verified! ({detections_verified}/{TOTAL_DETECTIONS_TO_FIND})")
                        print(f"  -> Distance: {distance} cm")
                        print(f"  -> Stepper Angle: {current_stepper_angle:.1f}°")
                        print(f"  -> Servo Angle: {servo_angle}°")
                        print("="*40)
                        
                        # Brief pause between verifications
                        time.sleep(0.5)
                        consecutive_detections = 0
                        
            except queue.Empty:
                pass
                
            # Reverse direction at sweep ends with smaller steps
            if stepper_steps_taken >= ASSURANCE_STEPS:
                stepper_steps_taken = 0
                stepper_direction_cw = not stepper_direction_cw
                GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)
                consecutive_detections = 0
        
        print("\nAssurance scan complete! All targets verified.")
        print("\nVerified target positions:")
        for i, point in enumerate(detection_points, 1):
            print(f"Target {i}:")
            print(f"  -> Stepper Angle: {point['stepper']:.1f}°")
            print(f"  -> Servo Angle: {point['servo']}°")
            print(f"  -> Distance: {point['distance']} cm")
        

        # =================================================================
        # 4. TRAJECTORY FOLLOWING AND UPDATE
        # =================================================================
        print("\nStarting Trajectory Following phase...")

        # Trajectory parameters
        TRAJECTORY_STEPPER_SWEEP = 10  # Narrower sweep for tracking
        TRAJECTORY_SERVO_STEP = 1.5    # Faster servo movement
        UPDATE_THRESHOLD = 5           # Minimum change to update trajectory
        
        # Initialize trajectory
        current_trajectory = detection_points.copy()
        tracking_active = True
        
        # Movement parameters
        tracking_steps = int((TRAJECTORY_STEPPER_SWEEP / 360.0) * STEPS_PER_REVOLUTION)
        stepper_steps_taken = 0
        last_detection_time = time.time()
        
        try:
            while tracking_active:
                # Quick zig-zag movement
                GPIO.output(STEP_PIN, GPIO.HIGH)
                time.sleep(STEPPER_PULSE_DELAY / 2)  # Faster stepper movement
                GPIO.output(STEP_PIN, GPIO.LOW)
                time.sleep(STEPPER_PULSE_DELAY / 2)
                stepper_steps_taken += 1
                
                # Faster servo movement
                if time.time() - last_servo_update > (SERVO_UPDATE_INTERVAL / 3):
                    last_servo_update = time.time()
                    if servo_direction_up:
                        servo_angle += TRAJECTORY_SERVO_STEP
                        if servo_angle >= SERVO_SWEEP_END:
                            servo_direction_up = False
                    else:
                        servo_angle -= TRAJECTORY_SERVO_STEP
                        if servo_angle <= SERVO_SWEEP_START:
                            servo_direction_up = True
                    set_servo_angle(pi, servo_angle)
                
                try:
                    distance = lidar_data_queue.get_nowait()
                    
                    if distance < (average_distance * DETECTION_THRESHOLD_FACTOR):
                        consecutive_detections += 1
                    else:
                        consecutive_detections = 0
                        
                    if consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                        # Calculate current position
                        center_angle = 180
                        half_sweep = TRAJECTORY_STEPPER_SWEEP / 2.0
                        angle_offset = (stepper_steps_taken / tracking_steps) * TRAJECTORY_STEPPER_SWEEP
                        current_stepper_angle = (center_angle - half_sweep) + angle_offset if stepper_direction_cw else (center_angle + half_sweep) - angle_offset
                        
                        # Create new detection point
                        new_point = {
                            'stepper': current_stepper_angle,
                            'servo': servo_angle,
                            'distance': distance
                        }
                        
                        # Check if this point indicates a trajectory change
                        trajectory_updated = False
                        for i, tracked_point in enumerate(current_trajectory):
                            if (abs(tracked_point['stepper'] - current_stepper_angle) < 10 and 
                                abs(tracked_point['servo'] - servo_angle) < 10):
                                # Calculate if significant change occurred
                                distance_change = abs(tracked_point['distance'] - distance)
                                if distance_change > UPDATE_THRESHOLD:
                                    current_trajectory[i] = new_point
                                    trajectory_updated = True
                                    print("\nTrajectory point updated!")
                                    print(f"  -> Old distance: {tracked_point['distance']} cm")
                                    print(f"  -> New distance: {distance} cm")
                                    print(f"  -> Change: {distance_change:.1f} cm")
                                break
                        
                        if trajectory_updated:
                            # Update movement parameters based on new trajectory
                            last_detection_time = time.time()
                            consecutive_detections = 0
                            
                            # Send updated point to UI (placeholder)
                            print(f"Sending updated trajectory point to UI:")
                            print(f"  -> Stepper: {new_point['stepper']:.1f}°")
                            print(f"  -> Servo: {new_point['servo']}°")
                            print(f"  -> Distance: {new_point['distance']} cm")
                            
                except queue.Empty:
                    pass
                
                # Reverse direction at sweep ends
                if stepper_steps_taken >= tracking_steps:
                    stepper_steps_taken = 0
                    stepper_direction_cw = not stepper_direction_cw
                    GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)
                
                # Check if we should continue tracking
                if time.time() - last_detection_time > 10:  # 10 seconds timeout
                    tracking_active = False
                    print("\nTracking timeout - No significant changes detected.")
                
                # Allow for clean exit
                if keyboard.is_pressed('q'):  # You'll need to: import keyboard
                    tracking_active = False
                    print("\nTracking stopped by user.")
        
        except Exception as e:
            print(f"\nError during trajectory tracking: {str(e)}")
        
        print("\nTrajectory following complete.")
        print("\nFinal trajectory points:")
        for i, point in enumerate(current_trajectory, 1):
            print(f"Point {i}:")
            print(f"  -> Stepper Angle: {point['stepper']:.1f}°")
            print(f"  -> Servo Angle: {point['servo']}°")
            print(f"  -> Distance: {point['distance']} cm")


    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Cleaning up...")
        if 'lidar_thread' in locals():
            lidar_thread.cleanup()
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
