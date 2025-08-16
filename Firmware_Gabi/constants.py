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
SERVO_SWEEP_END = 70

# --- Detection Settings ---
DEFAULT_CALIBRATION_DISTANCE = 1200  # cm - adjust based on your environment
SENSOR_MAX = 1200
ANOMALY_FACTOR = 0.6  # Factor to determine if reading is anomalous
INITIAL_SWEEP_DETECTIONS_COUNT = 5

# --- Scanning Settings ---
SWEEP_RANGE = 10  # Degrees for each sweep