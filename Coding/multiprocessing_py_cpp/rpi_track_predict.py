# =================================================================
# Python Trajectory Tracking with Unscented Kalman Filter
#
# Purpose: This script calibrates, finds 3 initial points, and then
# enters a trajectory tracking mode using a 3D Unscented Kalman Filter
# to predict the target's non-linear path.
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
import math

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
ACQUISITION_STEPPER_DEGREES = 40
ACQUISITION_SERVO_START = 5
ACQUISITION_SERVO_END = 25
ACQUISITION_PULSE_DELAY = 0.0001

TRACKING_STEPPER_DEGREES = 20
TRACKING_SERVO_DEGREES = 10
TRACKING_PULSE_DELAY = 0.00005

# --- Calibration & Detection Settings ---
CALIBRATION_SWEEPS = 2
DETECTION_THRESHOLD_FACTOR = 0.8
DETECTION_CONFIDENCE_THRESHOLD = 5
TOTAL_POINTS_TO_FIND = 3

# --- Coordinate Conversion ---
def spherical_to_cartesian(az, el, dist):
    """Converts spherical coordinates (degrees, cm) to Cartesian (m)."""
    az_rad = math.radians(az)
    el_rad = math.radians(el)
    dist_m = dist / 100.0
    x = dist_m * math.cos(el_rad) * math.sin(az_rad)
    y = dist_m * math.cos(el_rad) * math.cos(az_rad)
    z = dist_m * math.sin(el_rad)
    return x, y, z

def cartesian_to_spherical(x, y, z):
    """Converts Cartesian coordinates (m) back to spherical (degrees, cm)."""
    dist_m = math.sqrt(x**2 + y**2 + z**2)
    el = math.degrees(math.atan2(z, math.sqrt(x**2 + y**2)))
    az = math.degrees(math.atan2(x, y))
    return az, el, dist_m * 100

# --- Unscented Kalman Filter Implementation ---
# Define the non-linear state transition function (constant velocity model)
def f_cv(x, dt):
    F = np.array([[1, dt, 0,  0, 0,  0],
                  [0,  1, 0,  0, 0,  0],
                  [0,  0, 1, dt, 0,  0],
                  [0,  0, 0,  1, 0,  0],
                  [0,  0, 0,  0, 1, dt],
                  [0,  0, 0,  0, 0,  1]])
    return F @ x

# Define the non-linear measurement function (Cartesian to Spherical)
def h_spherical(x):
    az, el, dist = cartesian_to_spherical(x[0], x[2], x[4])
    return np.array([az, el, dist])

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

    # --- UKF Setup ---
    dt = 0.1 # Time step
    points = MerweScaledSigmaPoints(n=6, alpha=.1, beta=2., kappa=-1)
    ukf = UnscentedKalmanFilter(dim_x=6, dim_z=3, dt=dt, hx=h_spherical, fx=f_cv, points=points)
    ukf.R = np.diag([0.5, 0.5, 1.0]) # Measurement noise [az, el, dist]
    ukf.Q = np.eye(6) * 0.01         # Process noise

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
            is_tracking = current_state == "TRACKING"
            stepper_sweep_deg = TRACKING_STEPPER_DEGREES if is_tracking else ACQUISITION_STEPPER_DEGREES
            servo_sweep_start = ACQUISITION_SERVO_START
            servo_sweep_end = ACQUISITION_SERVO_END
            pulse_delay = TRACKING_PULSE_DELAY if is_tracking else ACQUISITION_PULSE_DELAY
            steps_for_sweep = int((stepper_sweep_deg / 360.0) * STEPS_PER_REVOLUTION)

            if is_tracking:
                ukf.predict()
                predicted_az, predicted_el, _ = cartesian_to_spherical(ukf.x[0], ukf.x[2], ukf.x[4])
                stepper_sweep_start = predicted_az - (stepper_sweep_deg / 2)
                servo_sweep_start = predicted_el - (TRACKING_SERVO_DEGREES / 2)
                servo_sweep_end = predicted_el + (TRACKING_SERVO_DEGREES / 2)
            
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

            try:
                distance = lidar_data_queue.get_nowait()
                
                if current_state == "CALIBRATING":
                    calibration_distances.append(distance)
                
                elif distance < (average_distance * DETECTION_THRESHOLD_FACTOR):
                    consecutive_detections += 1
                    if consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                        angle_offset = (stepper_steps_taken / steps_for_sweep) * stepper_sweep_deg
                        stepper_center = 180 if not is_tracking else predicted_az
                        stepper_start = stepper_center - (stepper_sweep_deg / 2)
                        current_stepper_angle = stepper_start + angle_offset if stepper_direction_cw else stepper_start + stepper_sweep_deg - angle_offset

                        point_data = {"dist": distance, "sa": current_stepper_angle, "sv": servo_angle}
                        
                        if is_tracking:
                            print(f"TRACKING UPDATE: Dist:{distance}cm, Stepper:{current_stepper_angle:.1f}°, Servo:{servo_angle}°")
                            ukf.update(np.array([current_stepper_angle, servo_angle, distance]))
                        else:
                            detected_points.append(point_data)
                            print(f"TARGET POINT {len(detected_points)} DETECTED! Dist:{distance}cm, Stepper:{current_stepper_angle:.1f}°, Servo:{servo_angle}°")
                            if len(detected_points) >= TOTAL_POINTS_TO_FIND:
                                print("\n--- INITIALIZING UKF WITH FIRST 3 POINTS ---")
                                for p in detected_points:
                                    x, y, z = spherical_to_cartesian(p["sa"], p["sv"], p["dist"])
                                    ukf.x = np.array([x, 0, y, 0, z, 0])
                                    ukf.update(np.array([p["sa"], p["sv"], p["dist"]]))
                                current_state = states[2]
                else:
                    consecutive_detections = 0
            except queue.Empty:
                pass

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
                            current_state = states[1]
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
    
