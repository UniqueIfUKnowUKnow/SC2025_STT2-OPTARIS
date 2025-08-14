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

    lidar_data_queue = queue.Queue()
    lidar_thread = LidarReader(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, lidar_data_queue)
    lidar_thread.start()

    elements = parse_tle([satellite_name, line1, line2])

    # --- State Machine and Variables ---
    states = ["CALIBRATING", "SCANNING", "DETECTED"]
    current_state = states[0]
    print(f"Current State: {current_state}")

    # Motor state variables
    stepper_steps_taken = 0
    GPIO.output(DIR_PIN, GPIO.HIGH)
    servo_angle = SERVO_SWEEP_START
    set_servo_angle(pi, servo_angle)

    try:
        #Mapping environment
        #calibration_data = calibrate_environment(pi, lidar_data_queue)
        #print(calibration_data)
        #save_calibration_data(calibration_data)

        print("Scanning complete.")
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Cleaning up...")
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        GPIO.cleanup()
        reset_stepper_pos(stepper_steps_taken)
        

if __name__ == '__main__':
    main()

