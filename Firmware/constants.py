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
# LiDAR config
LIDAR_SERIAL_PORT = '/dev/ttyS0'
LIDAR_BAUD_RATE = 115200

# --- Motor & Movement Settings ---
STEPS_PER_REVOLUTION = 6400
STEPPER_PULSE_DELAY = 0.0001
MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500

# Stepper and Servo Sweep Settings
STEPPER_SWEEP_DEGREES = 360
STEPS_FOR_SWEEP = int((STEPPER_SWEEP_DEGREES / 360.0) * STEPS_PER_REVOLUTION)
SERVO_SWEEP_START = 0
SERVO_SWEEP_END = 30


# Initial Detection Sweeps Settings
DEFAULT_CALIBRATION_DISTANCE = 1000  # cm - adjust based on your environment
SENSOR_MAX = 1000
ANOMALY_MAX_RADIUS = 3.0 # max degree range in which to average points
SWEEP_RANGE = 10
ANOMALY_FACTOR = 0.7
TILT_EXPANSION_FACTOR = 1
AZI_EXPANSION_FACTOR = 0.5
MISS_UNCERTAINTY_GROWTH_RATE = 0.08

# Tracking Prediction Settings

#Time step
DT = 0.5

# Phase-space filter parameters (tuned for phase dynamics)
ALPHA_PHASE = 0.2   # Position correction gain
BETA_PHASE = 0.1   # Velocity correction gain

# Test TLE
satellite_name = "ISS (ZARYA)"
line1 = "1 25544U 98067A   24225.51782528  .00016717  00000-0  30199-3 0  9992"
line2 = "2 25544  51.6426  95.0936 0007310  -1.0000  60.6990 15.50209251454141"

AZIMUTH_AMOUNT = -5
TILT_AMOUNT = 0
