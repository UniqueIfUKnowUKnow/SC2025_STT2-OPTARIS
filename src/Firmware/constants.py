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
ANOMALY_MAX_RADIUS = 4.0 # max degree range in which to average points
SWEEP_RANGE = 10
ANOMALY_FACTOR = 0.4
TILT_EXPANSION_FACTOR = 0.5
AZI_EXPANSION_FACTOR = 0.5
MISS_UNCERTAINTY_GROWTH_RATE = 0.08

# Tracking Prediction Settings

#Time step
DT = 0.5

# Phase-space filter parameters (tuned for phase dynamics)
ALPHA_PHASE = 0.45   # Position correction gain
BETA_PHASE = 0.2  # Velocity correction gain

# Test TLE
satellite_name = "Star link 1010"
line1 = "1 44716U 19074D   25232.33335648  .00163869  00000+0  13147-2 0  9992"
line2 = "2 44716  15.0556 334.9223 0010420 301.5321  72.1533 15.699406763190588"

AZIMUTH_AMOUNT = 10
TILT_AMOUNT = 0