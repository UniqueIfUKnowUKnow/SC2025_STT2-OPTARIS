# constants.py
# All constant definitions for the LiDAR scanning system
import numpy as np
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
SERVO_SWEEP_END = 30



# Initial Detection Sweeps Settings
DEFAULT_CALIBRATION_DISTANCE = 1200  # cm - adjust based on your environment
SENSOR_MAX = 1200
ANOMALY_MAX_RADIUS = 5.0 # max degree range in which to average points
SWEEP_RANGE = 10
ANOMALY_FACTOR = 0.6
INITIAL_SWEEP_DETECTIONS_COUNT = 5


# Tracking Prediction Settings
DT = 1
KALMAN_ERROR_RANGE = 1
START_TIME = 0

#Azimuth
ALPHA_THETA = 0.5
BETA_THETA = 0.2
# Elevation
ALPHA_PHI = 0.4
BETA_PHI = 0.15
#Residual smoothing (uncertainty; EWMA factor) 
LAMBDA = 0.95 
#Process floor for variance (convert to radians²)
Q_THE = (np.radians(0.2))**2
Q_PHI = (np.radians(0.2))**2 
#Scan sizing (≈99% capture if residuals are Gaussian) 
KA = 2.5 
#Minimum half-width of scan (≥ half of 2° FoV) 
W_MIN = np.radians(1.25) 
#Scan step  (≤ 1° to guarantee FoV overlap)
STEP_DEG = np.radians(1.0)
#Velocity caps (safety): 
W_THETA_MAX = np.radians(170) 
W_PHI_MAX = np.radians(60)
#Time step
DT = 0.5

# Test TLE
satellite_name = "ISS (ZARYA)"
line1 = "1 25544U 98067A   24225.51782528  .00016717  00000-0  30199-3 0  9992"
line2 = "2 25544  51.6426  95.0936 0007310  00.0000  60.6990 15.50209251454141"

AZIMUTH_AMOUNT = 10
TILT_AMOUNT = 0