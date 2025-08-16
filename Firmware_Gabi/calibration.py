# calibration.py
# Simplified environmental calibration for drone tracking
import RPi.GPIO as GPIO
import time
import threading
import queue
import csv
from lidar_reader import LidarReader
from constants import *
from stepper_setup import setup_stepper_gpio
from move_motors import *
from datetime import datetime

def calibrate_environment(pi, lidar_data_queue):
    """
    Performs systematic calibration by sweeping stepper motor through 360 degrees,
    then incrementing servo by 5 degrees and repeating. Collects distance data
    with corresponding azimuth and elevation positions.
    
    Returns:
        list: Array of [average_distance, azimuth, elevation] measurements
    """
    print("Starting environmental calibration...")
    
    # Initialize calibration data array
    calibration_data = []
    
    # Reset motors to starting position [0,0]
    print("Moving motors to starting position [0,0]...")
    
    # Set servo to 0 degrees (elevation)
    current_elevation = 0
    set_servo_angle(pi, current_elevation)
    time.sleep(1)  # Allow servo to reach position
    
    # Set stepper to 0 degrees (azimuth)
    current_physical_azimuth = 0.0
    stepper_steps_taken = 0
    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    # Calculate servo range - use larger increments for faster calibration
    elevation_positions = list(range(SERVO_SWEEP_START, SERVO_SWEEP_END, 5))
    
    # Calculate azimuth increments - use 2 degree increments for reasonable resolution
    azimuth_increments = [round(x * 2.0, 1) for x in range(0, int((STEPPER_SWEEP_DEGREES/2.0)+1))]
    
    for elevation in elevation_positions:
        print(f"Calibrating at elevation: {elevation}°")
        
        # Move servo to current elevation
        set_servo_angle(pi, elevation)
        time.sleep(0.5)  # Allow servo to settle
        
        # Dictionary to collect readings for each azimuth position
        azimuth_readings = {azimuth: [] for azimuth in azimuth_increments}
        
        # Sweep stepper motor forward through 360 degrees
        target_steps = STEPS_FOR_SWEEP
        GPIO.output(DIR_PIN, GPIO.HIGH)
        
        while stepper_steps_taken < target_steps:
            stepper_steps_taken += 1
            
            # Step the stepper motor
            stepper_step()
            
            # Update physical position
            current_physical_azimuth = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
            
            # Find the nearest 2-degree increment
            nearest_azimuth = round(current_physical_azimuth / 2.0) * 2.0
            if nearest_azimuth > 360:
                nearest_azimuth = 360.0

            # Collect LiDAR data if available
            try:
                distance = lidar_data_queue.get_nowait()
                if distance > 0 and distance < SENSOR_MAX:
                    if nearest_azimuth in azimuth_readings:
                        azimuth_readings[nearest_azimuth].append(distance)
            except queue.Empty:
                pass
        
        # Reset stepper for next elevation
        stepper_steps_taken = 0
        
        # Process collected readings for this elevation
        for azimuth in azimuth_increments:
            readings = azimuth_readings[azimuth]
            if readings:  # Only store if we have readings for this position
                # Calculate average distance
                average_distance = sum(readings) / len(readings)
                calibration_data.append([average_distance, azimuth, elevation])
                print(f"  Azimuth {azimuth}°: {len(readings)} readings, avg = {average_distance:.1f}cm")
    
    # Return motors to starting position [0,0]
    print("Returning motors to starting position [0,0]...")
    
    # Return servo to 0 degrees
    set_servo_angle(pi, 0)
    
    # Return stepper to 0 degrees
    reset_stepper_pos(stepper_steps_taken)
    
    print(f"Calibration complete! Collected {len(calibration_data)} measurements.")
    print("Motors returned to starting position [0,0]")
    
    return calibration_data

def save_calibration_data(calibration_data):
    """
    Save calibration data to CSV file with timestamp.
    
    Args:
        calibration_data: List of [distance, azimuth, elevation] measurements
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Save as CSV file
    csv_filename = f"lidar_calibration_{timestamp}.csv"
    try:
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Distance_cm', 'Azimuth_deg', 'Elevation_deg'])  # Header
            for row in calibration_data:
                writer.writerow([f"{row[0]:.1f}", f"{row[1]:.1f}", row[2]])
        print(f"✓ Calibration data saved to: {csv_filename}")
    except Exception as e:
        print(f"✗ Error saving CSV: {e}")
