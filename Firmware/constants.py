# constants.py
# All constant definitions for the LiDAR scanning system

# --- Pin & Port Configuration (BCM numbering) ---
DIR_PIN    = 26
STEP_PIN   = 13
ENABLE_PIN = 23
RESET_PIN  = 19
M0_PIN     = 0
M1_PIN     = 5
M2_PIN     = 6

SERVO_PIN  = 2

LIDAR_SERIAL_PORT = '/dev/ttyS0'
LIDAR_BAUD_RATE = 115200

# --- Motor & Movement Settings ---
STEPS_PER_REVOLUTION = 6400
STEPPER_PULSE_DELAY = 0.0001
MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500
SERVO_UPDATE_INTERVAL = 0.01

# Stepper and Servo Sweep Settings
STEPPER_SWEEP_DEGREES = 360
STEPS_FOR_SWEEP = int((STEPPER_SWEEP_DEGREES / 360.0) * STEPS_PER_REVOLUTION)
SERVO_SWEEP_START = 0
SERVO_SWEEP_END = 10

# --- Calibration & Detection Settings ---
CALIBRATION_SWEEPS = 2
DETECTION_THRESHOLD_FACTOR = 0.8
SENSOR_MAX = 1300
# --- FIX: New constant to require multiple consecutive detections. ---
# This prevents false positives from single noisy readings.
DETECTION_CONFIDENCE_THRESHOLD = 5

# --- Assurance Scan Settings ---
ZIG_ZAG_STEP_SIZE = 2  # Degrees per step in zigzag pattern
ZIG_ZAG_ELEVATION_INCREMENT = 1  # Degrees to increment elevation each sweep
POINTS_NEEDED = 3  # Total number of points needed for trajectory
MAX_POINTS_MEMORY = 100  # Maximum number of points to store for trajectory

# --- Trajectory Following Settings ---
TRAJECTORY_UPDATE_THRESHOLD = 5.0  # cm - Minimum change needed to update trajectory
NARROW_SCAN_WIDTH = 20  # Degrees - Width of scan when following trajectory
TRACKING_SPEED_MULTIPLIER = 1.5  # Speed increase for tracking mode

# --- Initial Detection Settings
ANOMALY_THRESHOLD_FACTOR = 0.7
CONSECUTIVE_DETECTIONS_REQUIRED = 5
MAX_DISTANCE_CHANGE = 50
INITIAL_SWEEP_AZIMUTH_START = -10
INITIAL_SWEEP_AZIMUTH_END = 10
INITIAL_SWEEP_TILT_START = 0
INITIAL_SWEEP_TILT_END = 0


# -----------------------
satellite_name = "ISS (ZARYA)"
line1 = "1 25544U 98067A   24225.51782528  .00016717  00000-0  30199-3 0  9992"
line2 = "2 25544  51.6426  95.0936 0007310  21.6487  60.6990 15.50209251454141"
