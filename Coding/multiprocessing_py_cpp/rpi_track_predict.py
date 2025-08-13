# =================================================================
# Python Trajectory Tracking with Kalman Filter
#
# Purpose: This script calibrates, finds 3 initial points, and then
# enters a trajectory tracking mode using a Kalman filter to predict
# the target's path and guide a narrow, fast scanning pattern.
# =================================================================

# --- Standard Library Imports ---
import RPi.GPIO as GPIO
import pigpio
import time
import serial
import threading
import queue
import statistics
import numpy as np

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
MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500
SERVO_UPDATE_INTERVAL = 0.01

# --- Scanning & Tracking Sweep Settings ---
# Wide sweep for initial acquisition
ACQUISITION_STEPPER_DEGREES = 40
ACQUISITION_SERVO_START = 5
ACQUISITION_SERVO_END = 25
ACQUISITION_PULSE_DELAY = 0.0001

# Narrow, fast sweep for tracking
TRACKING_STEPPER_DEGREES = 20
TRACKING_SERVO_DEGREES = 10
TRACKING_PULSE_DELAY = 0.00005 # Faster pulse

# --- Calibration & Detection Settings ---
CALIBRATION_SWEEPS = 2
DETECTION_THRESHOLD_FACTOR = 0.8
DETECTION_CONFIDENCE_THRESHOLD = 5
TOTAL_POINTS_TO_FIND = 3

# --- Kalman Filter for Angular Tracking ---
class KalmanTracker:
    """A 1D Kalman filter to track an angle and its velocity."""
    def __init__(self, process_noise=0.01, measurement_noise=0.1, dt=0.1):
        # State: [angle, angular_velocity]
        self.x = np.array([[0.0], [0.0]])
        # State Covariance Matrix
        self.P = np.eye(2)
        # State Transition Matrix
        self.F = np.array([[1, dt], [0, 1]])
        # Measurement Matrix
        self.H = np.array([[1, 0]])
        # Measurement Noise Covariance
        self.R = np.array([[measurement_noise]])
        # Process Noise Covariance
        self.Q = np.array([[process_noise, 0], [0, process_noise]])

    def predict(self):
        """Predicts the next state."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[0, 0]

    def update(self, measurement):
        """Updates the filter with a new measurement."""
        y = measurement - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K * y
        self.P = (np.eye(2) - K @ self.H) @ self.P

# --- LiDAR Reader Thread (Unchanged) ---
class LidarReader(threading.Thread):
    def __init__(self, port, baudrate, data_queue):
        super().__init__(daemon=True)
        self.port, self.baudrate, self.data_queue = port, baudrate, data_queue
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.ser.flushInput()
        self.frame_header = 0x59
        print("LiDAR Reader thread initialized.")

    def run(self):
        while True:
            if self.ser.read(1) == b'\x59' and self.ser.read(1) == b'\x59':
                frame = b'\x59\x59' + self.ser.read(7)
                if len(frame) == 9 and sum(frame[:-1]) & 0xFF == frame[8]:
                    distance_cm = frame[2] + (frame[3] << 8)
                    if distance_cm > 0: self.data_queue.put(distance_cm)

# --- Setup and Control Functions (Unchanged) ---
def setup_stepper_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pins = [DIR_PIN, STEP_PIN, ENABLE_PIN, RESET_PIN, M0_PIN, M1_PIN, M2_PIN]
    for pin in pins: GPIO.setup(pin, GPIO.OUT)
    GPIO.output(ENABLE_PIN, GPIO.LOW)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    GPIO.output(M0_PIN, GPIO.HIGH)
    GPIO.output(M1_PIN, GPIO.HIGH)
    GPIO.output(M2_PIN, GPIO.HIGH)

def set_servo_angle(pi, angle):
    pulse_width = MIN_PULSE_WIDTH + (np.clip(angle, 0, 180) / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

# --- Main Application ---
def main():
    # --- Setup ---
    setup_stepper_gpio()
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon."); return

    lidar_data_queue = queue.Queue()
    lidar_thread = LidarReader(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, lidar_data_queue)
    lidar_thread.start()

    # --- State Machine and Variables ---
    states = ["CALIBRATING", "ASSURANCE_SCAN", "TRACKING", "FINISHED"]
    current_state = states[0]
    print(f"Current State: {current_state}")

    # Motor state
    stepper_steps_taken = 0
    stepper_direction_cw = True
    GPIO.output(DIR_PIN, GPIO.HIGH)
    servo_angle = ACQUISITION_SERVO_START
    servo_direction_up = True
    set_servo_angle(pi, servo_angle)
    last_servo_update = time.time()
    
    # Data collection
    calibration_distances, detected_points = [], []
    sweeps_completed, average_distance = 0, 0
    consecutive_detections = 0

    # Kalman Filter instances
    stepper_kalman = KalmanTracker()
    servo_kalman = KalmanTracker()

    try:
        while current_state != "FINISHED":
            # Determine current sweep parameters based on state
            is_tracking = current_state == "TRACKING"
            stepper_sweep_deg = TRACKING_STEPPER_DEGREES if is_tracking else ACQUISITION_STEPPER_DEGREES
            servo_sweep_start = ACQUISITION_SERVO_START
            servo_sweep_end = ACQUISITION_SERVO_END
            pulse_delay = TRACKING_PULSE_DELAY if is_tracking else ACQUISITION_PULSE_DELAY
            steps_for_sweep = int((stepper_sweep_deg / 360.0) * STEPS_PER_REVOLUTION)

            # --- Dynamic Center Point for Tracking ---
            if is_tracking:
                # Predict the center of the sweep
                predicted_stepper_center = stepper_kalman.predict()
                predicted_servo_center = servo_kalman.predict()
                # Define the sweep range around the predicted center
                stepper_sweep_start = predicted_stepper_center - (stepper_sweep_deg / 2)
                servo_sweep_start = predicted_servo_center - (TRACKING_SERVO_DEGREES / 2)
                servo_sweep_end = predicted_servo_center + (TRACKING_SERVO_DEGREES / 2)
            
            # --- Motor Control ---
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(pulse_delay)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(pulse_delay)
            stepper_steps_taken += 1

            if time.time() - last_servo_update > SERVO_UPDATE_INTERVAL:
                last_servo_update = time.time()
                if servo_direction_up:
                    servo_angle += 1
                    if servo_angle >= servo_sweep_end: servo_direction_up = False
                else:
                    servo_angle -= 1
                    if servo_angle <= servo_sweep_start: servo_direction_up = True
                set_servo_angle(pi, servo_angle)

            # --- LiDAR Data & State Logic ---
            try:
                distance = lidar_data_queue.get_nowait()
                
                if current_state == "CALIBRATING":
                    calibration_distances.append(distance)
                
                elif distance < (average_distance * DETECTION_THRESHOLD_FACTOR):
                    consecutive_detections += 1
                    if consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                        # Calculate current angles
                        angle_offset = (stepper_steps_taken / steps_for_sweep) * stepper_sweep_deg
                        stepper_center = 180 if not is_tracking else predicted_stepper_center
                        stepper_start = stepper_center - (stepper_sweep_deg / 2)
                        current_stepper_angle = stepper_start + angle_offset if stepper_direction_cw else stepper_start + stepper_sweep_deg - angle_offset

                        point_data = {"dist": distance, "sa": current_stepper_angle, "sv": servo_angle}
                        
                        if is_tracking:
                            print(f"TRACKING UPDATE: Dist:{distance}cm, Stepper:{current_stepper_angle:.1f}°, Servo:{servo_angle}°")
                            stepper_kalman.update(current_stepper_angle)
                            servo_kalman.update(servo_angle)
                        else: # Assurance Scan
                            detected_points.append(point_data)
                            print(f"TARGET POINT {len(detected_points)} DETECTED! Dist:{distance}cm, Stepper:{current_stepper_angle:.1f}°, Servo:{servo_angle}°")
                            if len(detected_points) >= TOTAL_POINTS_TO_FIND:
                                print("\n--- INITIAL POINTS ACQUIRED, STARTING TRAJECTORY TRACKING ---")
                                # Initialize Kalman filters with the first 3 points
                                for p in detected_points:
                                    stepper_kalman.update(p["sa"])
                                    servo_kalman.update(p["sv"])
                                current_state = states[2] # Switch to TRACKING
                else:
                    consecutive_detections = 0
            except queue.Empty:
                pass

            # --- Sweep & State Transition ---
            if stepper_steps_taken >= steps_for_sweep:
                stepper_steps_taken = 0
                stepper_direction_cw = not stepper_direction_cw
                GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)
                consecutive_detections = 0

                if current_state == "CALIBRATING":
                    sweeps_completed += 0.5
                    print(f"Calibration sweep {int(sweeps_completed * 2)} of {CALIBRATION_SWEEPS * 2} halfs completed...")
                    if sweeps_completed >= CALIBRATION_SWEEPS:
                        if calibration_distances:
                            average_distance = statistics.mean(calibration_distances)
                            print(f"\nCALIBRATION COMPLETE. Avg Dist: {average_distance:.2f} cm\n")
                            current_state = states[1] # Switch to ASSURANCE_SCAN
                            print(f"Current State: {current_state} (Finding {TOTAL_POINTS_TO_FIND} initial points)")
                        else:
                            print("Calibration Failed."); current_state = states[3]

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Cleaning up...")
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
