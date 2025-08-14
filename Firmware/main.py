# --- Standard Library Imports ---
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import pigpio            # For stable, hardware-based PWM for the servo
import time              # For creating delays
import serial            # For reading data from the LiDAR's serial port
import threading         # To run the LiDAR reader in the background
import queue             # For thread-safe data sharing between threads
from lidar_reader import LidarReader
from constants import *
from stepper_setup import setup_stepper_gpio
from move_motors import *
from calibration import *
from tle_processing import parse_tle
from initial_sweep import sweep_scan_for_anomaly

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
    GPIO.output(DIR_PIN, GPIO.HIGH)
    

    servo_angle = SERVO_SWEEP_START
    set_servo_angle(pi, servo_angle)


    lidar_data_queue = queue.Queue()
    lidar_thread = LidarReader(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, lidar_data_queue)
    lidar_thread.start()

    tle_data = parse_tle([satellite_name, line1, line2])
    
    # --- State Machine and Variables ---
    states = ["CALIBRATING", "SCANNING", "DETECTED"]
    current_state = states[0]
    stepper_steps = 0
    current_azimuth = 0
    current_elevation = 0
    anomaly_detected = False

    try:
        while True:
            if current_state == "CALIBRATING":
                print("Calibrating sensors...")

                # Mapping environment
                calibration_data = calibrate_environment(pi, lidar_data_queue)
                print(calibration_data)
                save_calibration_data(calibration_data)

                # Moving to right of ascending node
                current_azimuth, current_elevation, stepper_steps = move_to_polar_position(pi, tle_data["arg_perigee_deg"], 0, stepper_steps)

                calibration_done = True
                if calibration_done:
                    current_state = states[1]

            elif current_state == "SCANNING":
                print("Scanning area...")
                # scanning code here
                anomaly_detected, anomaly_positions, stepper_steps, current_elevation, current_azimuth = sweep_scan_for_anomaly
                (pi, lidar_data_queue, calibration_data, 
                    stepper_steps, current_elevation,
                    AZIMUTH_SWEEP_RANGES, ELEVATION_SWEEP_RANGES,
                    ANOMALY_THRESHOLD_FACTOR,
                    CONSECUTIVE_DETECTIONS_REQUIRED,
                    MAX_DISTANCE_CHANGE)
                print(anomaly_detected)

                target_detected = True
                if target_detected:
                    current_state = states[2]

            elif current_state == "DETECTED":
                print("Target detected!")
                # detection handling code here

                break
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Cleaning up...")
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        GPIO.cleanup()
        reset_stepper_pos(stepper_steps)
        

if __name__ == '__main__':
    main()

