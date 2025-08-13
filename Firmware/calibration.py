# --- Standard Library Imports ---
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import pigpio            # For stable, hardware-based PWM for the servo
import time              # For creating delays
import serial            # For reading data from the LiDAR's serial port
import threading         # To run the LiDAR reader in the background
import queue             # For thread-safe data sharing between threads
import statistics        # For easily calculating the average distance
from lidar_reader import LidarReader
from constants import *
from stepper_setup import setup_stepper_gpio
from move_servo import *


def calibrate_environment(pi, lidar_data_queue):
    """
    Performs systematic calibration by sweeping stepper motor through 180 degrees,
    then incrementing servo by 2 degrees and repeating. Collects distance data
    with corresponding azimuth and elevation positions.
    
    Returns:
        list: Array of [distance, azimuth, elevation] measurements
    """
    print("Starting calibration...")
    
    # Initialize calibration data array
    calibration_data = []
    
    # Reset motors to starting position [0,0]
    print("Moving motors to starting position [0,0]...")
    
    # Set servo to 0 degrees (elevation)
    current_elevation = 0
    set_servo_angle(pi, current_elevation)
    time.sleep(1)  # Allow servo to reach position
    
    # Set stepper to 0 degrees (azimuth) - assuming we start at 0
    current_azimuth = 0
    stepper_direction_cw = True
    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    # Calculate servo range: 0 to 140 degrees in 2-degree increments
    elevation_positions = list(range(SERVO_SWEEP_START, SERVO_SWEEP_END, 2))  # [+0, +2, +4, ..., 160]
    stepper_steps_taken = 0
    current_azimuth = 0
    for elevation in elevation_positions:
        print(f"Calibrating at elevation: {elevation}°")
        
        # Move servo to current elevation
        set_servo_angle(pi, elevation)
        time.sleep(0.5)  # Allow servo to settle
        
        # Perform 180-degree stepper sweep
        if elevation % 4 == 0: 
            GPIO.output(DIR_PIN, GPIO.HIGH)
            while stepper_steps_taken < STEPS_FOR_SWEEP:
                    
                    stepper_steps_taken += 1
                    current_azimuth = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                
                    # Step the stepper motor
                    GPIO.output(STEP_PIN, GPIO.HIGH)
                    time.sleep(STEPPER_PULSE_DELAY)
                    GPIO.output(STEP_PIN, GPIO.LOW)
                    time.sleep(STEPPER_PULSE_DELAY)
                    
                    # Calculate current azimuth position
                    current_azimuth = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                    print (current_azimuth)
                    # Collect LiDAR data if available
                    try:
                        distance = lidar_data_queue.get_nowait()
                        # Store measurement with position data
                        calibration_data.append([distance, current_azimuth, elevation])
                    except queue.Empty:
                        pass
        else:
            GPIO.output(DIR_PIN, GPIO.LOW)
            while stepper_steps_taken > 0:
                
                stepper_steps_taken -= 1
                current_azimuth = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                # Step the stepper motor
                GPIO.output(STEP_PIN, GPIO.HIGH)
                time.sleep(STEPPER_PULSE_DELAY)
                GPIO.output(STEP_PIN, GPIO.LOW)
                time.sleep(STEPPER_PULSE_DELAY)
                    
                # Calculate current azimuth position
                current_azimuth = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                print (current_azimuth)
                # Collect LiDAR data if available
                try:
                    distance = lidar_data_queue.get_nowait()
                    # Store measurement with position data
                    calibration_data.append([distance, current_azimuth, elevation])
                except queue.Empty:
                    pass
        
    
    # Return motors to starting position [0,0]
    print("Returning motors to starting position [0,0]...")
    
    # Return servo to 0 degrees
    set_servo_angle(pi, 0)
    
    # Return stepper to 0 degrees (reverse all steps taken)
    GPIO.output(DIR_PIN, GPIO.LOW)  # Reverse direction
    for _ in range(stepper_steps_taken):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(STEPPER_PULSE_DELAY)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(STEPPER_PULSE_DELAY)
    
    # Reset direction for normal operation
    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    print(f"Calibration complete! Collected {len(calibration_data)} measurements.")
    print("Motors returned to starting position [0,0]")
    
    return calibration_data