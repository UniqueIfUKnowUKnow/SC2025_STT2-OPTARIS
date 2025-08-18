# =================================================================
# Python Custom Trajectory Tracker
#
# Purpose: This script calibrates the environment, acquires a target,
# and then uses a 3D Unscented Kalman Filter to track and
# predict the trajectory of a moving (orbiting) object.
# =================================================================

# --- Standard Library Imports ---
import RPi.GPIO as GPIO
import pigpio
import time
import serial
import threading
import queue
import statistics
import math
from collections import deque
import numpy as np

# --- Third-party library for Kalman Filter ---
# You may need to install this: pip install filterpy
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints

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
ACQUISITION_STEPPER_DEGREES = 180
ACQUISITION_SERVO_START = 0
ACQUISITION_SERVO_END = 30
ACQUISITION_PULSE_DELAY = 0.0001

# Narrow, fast sweep for tracking
TRACKING_STEPPER_DEGREES = 30
TRACKING_SERVO_DEGREES = 15
TRACKING_PULSE_DELAY = 0.00005 # Faster pulse

# --- Calibration & Detection Settings ---
CALIBRATION_SWEEPS = 2
DETECTION_THRESHOLD_FACTOR = 0.8
DETECTION_CONFIDENCE_THRESHOLD = 5
INITIAL_POINTS_TO_FIND = 3

# --- Coordinate Conversion ---
def spherical_to_cartesian(az, el, dist):
    """Converts spherical coordinates (degrees, cm) to Cartesian (cm)."""
    az_rad = math.radians(az)
    el_rad = math.radians(el)
    x = dist * math.cos(el_rad) * math.sin(az_rad)
    y = dist * math.cos(el_rad) * math.cos(az_rad)
    z = dist * math.sin(el_rad)
    return np.array([x, y, z])

def cartesian_to_spherical(point):
    """Converts a Cartesian point (cm) back to spherical (degrees, cm)."""
    x, y, z = point
    dist = math.sqrt(x**2 + y**2 + z**2)
    el = math.degrees(math.atan2(z, np.sqrt(x**2 + y**2)))
    az = math.degrees(math.atan2(y, x))
    if az < 0:
        az += 360
    return az, el, dist

# --- Unscented Kalman Filter Implementation ---
# Define the non-linear state transition function (constant velocity model)
def f_cv(x, dt):
    """State transition function for a 3D constant velocity model."""
    F = np.array([[1, dt, 0,  0, 0,  0],
                  [0,  1, 0,  0, 0,  0],
                  [0,  0, 1, dt, 0,  0],
                  [0,  0, 0,  1, 0,  0],
                  [0,  0, 0,  0, 1, dt],
                  [0,  0, 0,  0, 0,  1]])
    return F @ x

# Define the non-linear measurement function (Cartesian state to Spherical measurement)
def h_spherical(x):
    """Measurement function that converts a 3D Cartesian state (x,y,z)
       into a spherical measurement (azimuth, elevation, distance)."""
    az, el, dist = cartesian_to_spherical(np.array([x[0], x[2], x[4]]))
    return np.array([az, el, dist])

# --- LiDAR Reader Thread ---
class LidarReader(threading.Thread):
    def __init__(self, port, baudrate, data_queue):
        super().__init__(daemon=True)
        self.port, self.baudrate, self.data_queue = port, baudrate, data_queue
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.ser.flushInput()
        self.frame_header = 0x59
        print("LiDAR Reader thread initialized.")

    def run(self):
        """The main loop for the LiDAR reading thread."""
        while True:
            try:
                # Check for the double header 0x59 0x59
                if self.ser.in_waiting >= 9:
                    if self.ser.read(1) == b'\x59' and self.ser.read(1) == b'\x59':
                        # Read the rest of the 9-byte frame
                        frame = b'\x59\x59' + self.ser.read(7)
                        if len(frame) == 9 and sum(frame[:-1]) & 0xFF == frame[8]:
                            distance_cm = frame[2] + (frame[3] << 8)
                            if distance_cm > 0:
                                self.data_queue.put(distance_cm)
            except serial.SerialException as e:
                # Catch the exception to prevent the thread from crashing
                print(f"LiDAR Read Error: {e}. Re-syncing...")
                # Flush the buffer to try and recover from the error
                self.ser.flushInput()
                time.sleep(0.5) # Pause briefly before trying again
            # Add a small sleep to yield CPU time to other processes
            time.sleep(0.001)


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

    # --- UKF Setup ---
    dt = 0.1 # Assumed time step
    points = MerweScaledSigmaPoints(n=6, alpha=.1, beta=2., kappa=-1)
    ukf = UnscentedKalmanFilter(dim_x=6, dim_z=3, dt=dt, hx=h_spherical, fx=f_cv, points=points)
    ukf.x = np.zeros(6)
    ukf.R = np.diag([0.9, 0.9, 1.5]) # Measurement noise [az, el, dist]
    ukf.Q = np.eye(6) * 0.03

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

    try:
        while current_state != "FINISHED":
            # Determine current sweep parameters based on state
            is_tracking = current_state == "TRACKING"
            stepper_sweep_deg = TRACKING_STEPPER_DEGREES if is_tracking else ACQUISITION_STEPPER_DEGREES
            current_servo_sweep_start = ACQUISITION_SERVO_START
            current_servo_sweep_end = ACQUISITION_SERVO_END
            pulse_delay = TRACKING_PULSE_DELAY if is_tracking else ACQUISITION_PULSE_DELAY
            steps_for_sweep = int((stepper_sweep_deg / 360.0) * STEPS_PER_REVOLUTION)

            # --- Dynamic Center Point for Tracking ---
            if is_tracking:
                ukf.predict()
                predicted_az, predicted_el, _ = cartesian_to_spherical(np.array([ukf.x[0], ukf.x[2], ukf.x[4]]))
                # Clip servo angles to ensure they are within the valid 0-180 range.
                current_servo_sweep_start = np.clip(predicted_el - (TRACKING_SERVO_DEGREES / 2), 0, 180)
                current_servo_sweep_end = np.clip(predicted_el + (TRACKING_SERVO_DEGREES / 2), 0, 180)
            
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
                    if servo_angle >= current_servo_sweep_end: servo_direction_up = False
                else:
                    servo_angle -= 1
                    if servo_angle <= current_servo_sweep_start: servo_direction_up = True
                set_servo_angle(pi, servo_angle)

            # --- LiDAR Data & State Logic ---
            try:
                distance = lidar_data_queue.get_nowait()
                
                if current_state == "CALIBRATING":
                    calibration_distances.append(distance)
                
                elif distance < (average_distance * DETECTION_THRESHOLD_FACTOR):
                    consecutive_detections += 1
                    if consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                        stepper_center = 90 if not is_tracking else predicted_az
                        stepper_start = stepper_center - (stepper_sweep_deg / 2)
                        angle_offset = (stepper_steps_taken / steps_for_sweep) * stepper_sweep_deg
                        current_stepper_angle = stepper_start + angle_offset if stepper_direction_cw else stepper_start + stepper_sweep_deg - angle_offset

                        measurement = np.array([current_stepper_angle, servo_angle, distance])
                        
                        if is_tracking:
                            ukf.update(measurement)
                            print(f"TRACKING UPDATE: Dist:{distance}cm, Stepper:{current_stepper_angle:.1f}°, Servo:{servo_angle}°")
                        else: # Assurance Scan
                            detected_points.append(measurement)
                            print(f"TARGET POINT {len(detected_points)} DETECTED! Dist:{distance}cm, Stepper:{current_stepper_angle:.1f}°, Servo:{servo_angle}°")
                            if len(detected_points) >= INITIAL_POINTS_TO_FIND:
                                print("\n--- INITIALIZING UKF WITH FIRST 3 POINTS ---")
                                # Initialize UKF state with the first point
                                x, y, z = spherical_to_cartesian(detected_points[0][0], detected_points[0][1], detected_points[0][2])
                                ukf.x = np.array([x, 0, y, 0, z, 0])
                                # Update with all initial points
                                for p in detected_points:
                                    ukf.update(p)
                                current_state = states[2]
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
                            # --- FIX: Filter out outliers before calculating the average. ---
                            mean = statistics.mean(calibration_distances)
                            std_dev = statistics.stdev(calibration_distances)
                            # Only keep data points within 2 standard deviations of the mean.
                            filtered_distances = [d for d in calibration_distances if (mean - 2 * std_dev < d < mean + 2 * std_dev)]
                            
                            if filtered_distances:
                                average_distance = statistics.mean(filtered_distances)
                                print(f"\nCALIBRATION COMPLETE. Filtered Avg Dist: {average_distance:.2f} cm\n")
                            else:
                                # Fallback to raw average if filtering removes all data
                                average_distance = mean
                                print(f"\nCALIBRATION COMPLETE. Raw Avg Dist: {average_distance:.2f} cm\n")

                            # Clear the LiDAR queue after calibration.
                            with lidar_data_queue.mutex:
                                lidar_data_queue.queue.clear()
                            current_state = states[1]
                            print(f"Current State: {current_state} (Finding {INITIAL_POINTS_TO_FIND} initial points)")
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
