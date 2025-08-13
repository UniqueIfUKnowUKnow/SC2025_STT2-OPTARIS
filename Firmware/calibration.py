# --- Standard Library Imports ---
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import pigpio            # For stable, hardware-based PWM for the servo
import time              # For creating delays
import threading         # To run the LiDAR reader in the background
import queue             # For thread-safe data sharing between threads
import statistics        # For easily calculating the average distance
import csv               # For saving data to CSV files
import json              # For saving data to JSON files
from lidar_reader import LidarReader
from constants import *
from stepper_setup import setup_stepper_gpio
from move_motors import *
from datetime import datetime



def calibrate_environment(pi, lidar_data_queue):
    """
    Performs systematic calibration by sweeping stepper motor through 180 degrees,
    then incrementing servo by 2 degrees and repeating. Collects distance data
    with corresponding azimuth and elevation positions, averaging readings for
    each 2-degree azimuth increment.
    
    Returns:
        list: Array of [average_distance, azimuth, elevation] measurements at 2-degree increments
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

    current_physical_azimuth = 0.0  # Track actual physical position
    stepper_direction_cw = True
    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    # Calculate servo range
    elevation_positions = list(range(SERVO_SWEEP_START, SERVO_SWEEP_END, 2))
    
    # Calculate azimuth increments
    azimuth_increments = list(range(0, (STEPPER_SWEEP_DEGREES+1), 2))
    stepper_steps_taken = 0
    
    for elevation in elevation_positions:
        print(f"Calibrating at elevation: {elevation}°")
        
        # Move servo to current elevation
        set_servo_angle(pi, elevation)
        time.sleep(0.3)  # Allow servo to settle
        
        # Dictionary to collect readings for each azimuth position
        azimuth_readings = {azimuth: [] for azimuth in azimuth_increments}

        # Determine sweep direction for this elevation
        sweep_forward = ((SERVO_SWEEP_START - elevation) % 4 == 0)
        
        if sweep_forward:
            
            print("Forward sweep")
            # reset_stepper_pos(stepper_steps_taken)
            target_steps = STEPS_FOR_SWEEP
            GPIO.output(DIR_PIN, GPIO.HIGH)
            stepper_steps_taken = 0
            
            while stepper_steps_taken < target_steps:
                stepper_steps_taken += 1
                
                # Step the stepper motor
                stepper_step()
                
                # Update physical position
                current_physical_azimuth = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                
                # Find the nearest 2-degree increment
                nearest_azimuth = round(current_physical_azimuth / 2) * 2
                if nearest_azimuth > 360:
                    nearest_azimuth = 360

                # Collect LiDAR data if available
                try:
                    distance = lidar_data_queue.get_nowait()
                    if nearest_azimuth in azimuth_readings:
                        azimuth_readings[nearest_azimuth].append(distance)
                except queue.Empty:
                    pass
                    
        else:
            
            
            print("  Reverse sweep")
            # Reverse sweep: move from current position to 0°
            # reset_stepper_pos(stepper_steps_taken)
            target_steps = STEPS_FOR_SWEEP
            GPIO.output(DIR_PIN, GPIO.LOW)
            stepper_steps_taken = 0
            
            while stepper_steps_taken < target_steps:
                stepper_steps_taken += 1
                
                # Step the stepper motor
                stepper_step()
                
                # Update physical position
                current_physical_azimuth = 360 - (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                
                # Find the nearest 2-degree increment
                nearest_azimuth = round(current_physical_azimuth / 2) * 2
                if nearest_azimuth < 0:
                    nearest_azimuth = 0
                
                # Collect LiDAR data if available
                try:
                    distance = lidar_data_queue.get_nowait()
                    if nearest_azimuth in azimuth_readings:
                        azimuth_readings[nearest_azimuth].append(distance)
                except queue.Empty:
                    pass
        
        # Process collected readings for this elevation
        for azimuth in azimuth_increments:
            readings = azimuth_readings[azimuth]
            if readings:  # Only store if we have readings for this position
                # Calculate average distance, filtering out obvious outliers
                if len(readings) > 3:
                    mean_dist = statistics.mean(readings)
                    stdev_dist = statistics.stdev(readings)
                    filtered_readings = [r for r in readings if abs(r - mean_dist) <= 2 * stdev_dist]
                    if filtered_readings:
                        average_distance = statistics.mean(filtered_readings)
                    else:
                        average_distance = mean_dist
                else:
                    average_distance = statistics.mean(readings)
                
                # Store measurement with averaged distance and exact position
                calibration_data.append([average_distance, azimuth, elevation])
                print(f"  Azimuth {azimuth}°: {len(readings)} readings, avg = {average_distance:.1f}cm")
    
    # Return motors to starting position [0,0]
    print("Returning motors to starting position [0,0]...")
    
    # Return servo to 0 degrees
    set_servo_angle(pi, 0)
    
    # Return stepper to 0 degrees
    reset_stepper_pos(stepper_steps_taken)
    
    print(f"Calibration complete! Collected {len(calibration_data)} averaged measurements.")
    print("Motors returned to starting position [0,0]")
    
    return calibration_data

def save_calibration_data(calibration_data):
    """
    Save calibration data to multiple file formats with timestamp.
    
    Args:
        calibration_data: List of [distance, azimuth, elevation] measurements
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 1. Save as CSV file (easy to open in Excel/spreadsheet programs)
    csv_filename = f"lidar_calibration_{timestamp}.csv"
    try:
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Distance_cm', 'Azimuth_deg', 'Elevation_deg'])  # Header
            for row in calibration_data:
                writer.writerow([f"{row[0]:.1f}", row[1], row[2]])
        print(f"✓ CSV data saved to: {csv_filename}")
    except Exception as e:
        print(f"✗ Error saving CSV: {e}")
    # 2. Save as JSON file (good for programmatic access)
    json_filename = f"lidar_calibration_{timestamp}.json"
    try:
        json_data = {
            "timestamp": timestamp,
            "total_measurements": len(calibration_data),
            "measurements": [
                {
                    "distance_cm": round(row[0], 1),
                    "azimuth_deg": row[1],
                    "elevation_deg": row[2]
                }
                for row in calibration_data
            ]
        }
        with open(json_filename, 'w') as jsonfile:
            json.dump(json_data, jsonfile, indent=2)
        print(f"✓ JSON data saved to: {json_filename}")
    except Exception as e:
        print(f"✗ Error saving JSON: {e}")