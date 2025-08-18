# =================================================================
# Python Scanner with Predictive Tracking
#
# Purpose: This script calibrates the background, then scans for a
# target. Once at least five points are detected, it applies an
# ellipse fitting algorithm to predict the object's trajectory and
# switches to a predictive tracking mode.
# =================================================================

# --- Standard Library Imports ---
import RPi.GPIO as GPIO      # For direct control of stepper motor GPIO pins
import pigpio                # For stable, hardware-based PWM for the servo
import time                  # For creating delays
import serial                # For reading data from the LiDAR's serial port
import threading             # To run the LiDAR reader in the background
import queue                 # For thread-safe data sharing between threads
import statistics            # For easily calculating the average distance
import numpy as np           # For numerical operations
import math                  # For trigonometric functions

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
SERVO_UPDATE_INTERVAL = 0.01

# --- Stepper and Servo Sweep Settings ---
STEPPER_SWEEP_DEGREES = 40
STEPS_FOR_SWEEP = int((STEPPER_SWEEP_DEGREES / 360.0) * STEPS_PER_REVOLUTION)
SERVO_SWEEP_START = 5
SERVO_SWEEP_END = 25

# --- Calibration & Detection Settings ---
CALIBRATION_SWEEPS = 2
DETECTION_THRESHOLD_FACTOR = 0.8
DETECTION_CONFIDENCE_THRESHOLD = 5

# --- NEW: Constants for predictive tracking ---
POINTS_FOR_TRAJECTORY = 5
PREDICTION_STEP_ANGLE = 2.0  # Degrees to predict ahead

# --- LiDAR Reader Thread ---
class LidarReader(threading.Thread):
    """A dedicated thread that continuously reads data from the TFmini-S LiDAR."""
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

def fit_ellipse(points_cartesian):
    """
    Fits an ellipse to a set of Cartesian points using a least-squares method.
    Returns the coefficients A, B, C, D, E, F of the general conic equation.
    """
    x = points_cartesian[:, 0]
    y = points_cartesian[:, 1]
    
    D = np.vstack([x**2, x*y, y**2, x, y, np.ones(len(x))]).T
    
    U, S, V = np.linalg.svd(D, full_matrices=False)
    
    coeffs = V[-1, :]
    
    A, B, C = coeffs[0], coeffs[1], coeffs[2]
    if B**2 - 4*A*C >= 0:
        print("Warning: The fitted conic is not an ellipse.")
        return None
    
    return coeffs

def predict_next_point(coeffs, last_stepper_angle):
    """
    Uses the ellipse coefficients to predict the next position along the trajectory.
    This function finds the next point on the fitted ellipse, a small step ahead
    in terms of angle, and returns its corresponding polar coordinates.
    """
    A, B, C, D, E, F = coeffs
    
    next_stepper_angle = last_stepper_angle + PREDICTION_STEP_ANGLE
    
    angle_rad = np.radians(next_stepper_angle)
    
    a = A * np.cos(angle_rad)**2 + B * np.cos(angle_rad) * np.sin(angle_rad) + C * np.sin(angle_rad)**2
    b = D * np.cos(angle_rad) + E * np.sin(angle_rad)
    c = F
    
    discriminant = b**2 - 4*a*c
    
    if discriminant < 0:
        return None, None
    
    distance_cm = (-b + np.sqrt(discriminant)) / (2 * a)
    
    return next_stepper_angle, distance_cm

def move_stepper_to_angle(target_angle):
    """Moves the stepper motor to a specific angle."""
    current_stepper_angle = 180
    
    steps_to_move = int(((target_angle - current_stepper_angle) / 360.0) * STEPS_PER_REVOLUTION)
    
    if steps_to_move > 0:
        GPIO.output(DIR_PIN, GPIO.HIGH) # Clockwise
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)  # Counter-clockwise

    for _ in range(abs(steps_to_move)):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(STEPPER_PULSE_DELAY)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(STEPPER_PULSE_DELAY)

def main():
    """
    The main entry point of the program. It handles setup, state management,
    and the main control loop for the calibration, scanning, and tracking process.
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
    states = ["CALIBRATING", "SCANNING", "TRACKING", "FINISHED"]
    current_state = states[0]
    print(f"Current State: {current_state}")

    stepper_steps_taken = 0
    stepper_direction_cw = True
    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    servo_angle = SERVO_SWEEP_START
    servo_direction_up = True
    set_servo_angle(pi, servo_angle)
    last_servo_update = time.time()
    print(f"Servo sweep range set to {SERVO_SWEEP_START}° - {SERVO_SWEEP_END}°.")

    calibration_distances = []
    sweeps_completed = 0
    average_distance = 0
    
    consecutive_detections = 0
    
    trajectory_points = []
    
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

        stepper_steps_taken = 0
        with lidar_data_queue.mutex:
            lidar_data_queue.queue.clear()
        
        # =================================================================
        # 2. SCANNING & TRACKING PHASE
        # =================================================================
        while current_state != "FINISHED":
            if current_state == "SCANNING":
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

                        print("\n" + "="*40)
                        print(f"TARGET DETECTED!")
                        print(f"  -> Distance: {distance} cm (Avg: {average_distance:.2f} cm)")
                        print(f"  -> Stepper Angle: {current_stepper_angle:.1f}°")
                        print(f"  -> Servo Angle: {servo_angle}°")
                        print("="*40)
                        
                        detected_point = {
                            'stepper_angle': current_stepper_angle,
                            'servo_angle': servo_angle,
                            'distance': distance
                        }
                        trajectory_points.append(detected_point)
                        
                        if len(trajectory_points) >= POINTS_FOR_TRAJECTORY:
                            print(f"🎉 {len(trajectory_points)} points accumulated. Entering tracking mode.")
                            current_state = states[2]
                        else:
                            print(f"Pausing for 1 second, then resuming scan... ({len(trajectory_points)}/{POINTS_FOR_TRAJECTORY})")
                            time.sleep(1)
                            with lidar_data_queue.mutex:
                                lidar_data_queue.queue.clear()
                            consecutive_detections = 0
                
                except queue.Empty:
                    pass

                if stepper_steps_taken >= STEPS_FOR_SWEEP:
                    stepper_steps_taken = 0
                    stepper_direction_cw = not stepper_direction_cw
                    GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)
                    consecutive_detections = 0
            
            # =================================================================
            # 3. TRACKING PHASE
            # =================================================================
            elif current_state == "TRACKING":
                print("Predicting next position...")
                
                cartesian_points = []
                for point in trajectory_points:
                    stepper_rad = math.radians(point['stepper_angle'])
                    
                    x = point['distance'] * math.cos(stepper_rad)
                    y = point['distance'] * math.sin(stepper_rad)
                    cartesian_points.append([x, y])
                
                ellipse_coeffs = fit_ellipse(np.array(cartesian_points))
                
                if ellipse_coeffs is None:
                    print("Failed to fit an ellipse. Returning to scan mode.")
                    trajectory_points = []
                    current_state = states[1]
                    continue
                
                print("Ellipse fitted. Predicting future path...")
                
                last_stepper_angle = trajectory_points[-1]['stepper_angle']
                last_servo_angle = trajectory_points[-1]['servo_angle']
                
                predicted_stepper_angle, predicted_distance = predict_next_point(
                    ellipse_coeffs, last_stepper_angle)
                
                if predicted_stepper_angle is None:
                    print("Prediction failed. Returning to scan mode.")
                    trajectory_points = []
                    current_state = states[1]
                    continue

                print(f"Predicted target position: Stepper Angle {predicted_stepper_angle:.1f}°")
                
                move_stepper_to_angle(predicted_stepper_angle)
                
                time.sleep(0.5) 
                
                try:
                    new_distance = lidar_data_queue.get(timeout=0.1)
                    print(f"New reading at predicted position: {new_distance} cm")
                    
                    trajectory_points.append({
                        'stepper_angle': predicted_stepper_angle,
                        'servo_angle': last_servo_angle,  # Keep servo angle for now
                        'distance': new_distance
                    })
                    
                    if len(trajectory_points) > POINTS_FOR_TRAJECTORY:
                        trajectory_points.pop(0)

                except queue.Empty:
                    print("No new data received. Continuing tracking with last known points.")
                    
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Cleaning up...")
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()