# main.py
# Main drone tracking program implementing 4-step process
import RPi.GPIO as GPIO
import pigpio
import time
import serial
import threading
import queue
import csv
from datetime import datetime
from lidar_reader import LidarReader
from constants import *
from stepper_setup import setup_stepper_gpio
from move_motors import *
from calibration import *
from scanning import perform_scanning_sequence
from anomaly_detection import get_reference_distance, is_anomaly

def main():
    """
    Main drone tracking program implementing 4-step process:
    1. Environmental scan and save data
    2. Search scan for different points
    3. Track by scanning around detected points
    4. Continue tracking infinitely
    """
    print("=== Drone Tracking System ===")
    
    # --- Setup ---
    setup_stepper_gpio()
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Is it running?")
        return
    
    # Initialize motors
    GPIO.output(DIR_PIN, GPIO.HIGH)
    servo_angle = SERVO_SWEEP_START
    set_servo_angle(pi, servo_angle)
    
    # Initialize LiDAR
    lidar_data_queue = queue.Queue()
    lidar_thread = LidarReader(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, lidar_data_queue)
    lidar_thread.start()
    
    # --- State Machine ---
    current_state = "CALIBRATING"
    stepper_steps = 0
    current_azimuth = 0
    current_elevation = 0
    calibration_data = []
    last_detected_point = None
    
    try:
        while True:
            if current_state == "CALIBRATING":
                print("\n=== STEP 1: Environmental Calibration ===")
                print("Scanning environment to establish baseline...")
                
                # Perform environmental calibration
                calibration_data = calibrate_environment(pi, lidar_data_queue)
                print(f"Calibration complete! Collected {len(calibration_data)} measurements")
                
                # Save calibration data
                save_calibration_data(calibration_data)
                
                # Move to starting position for search
                current_azimuth, current_elevation, stepper_steps = move_to_polar_position(
                    pi, 0, 35, stepper_steps
                )
                
                current_state = "SEARCHING"
                print("Moving to search state...")
                
            elif current_state == "SEARCHING":
                print("\n=== STEP 2: Search Scan ===")
                print("Searching for drone targets...")
                
                # Perform search scanning
                current_azimuth, current_elevation, stepper_steps, anomaly_detected, anomaly_location = perform_scanning_sequence(
                    pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, stepper_steps
                )
                
                if anomaly_detected:
                    print(f"Target detected at: {anomaly_location}")
                    last_detected_point = anomaly_location
                    current_state = "TRACKING"
                    print("Moving to tracking state...")
                else:
                    # Move to next search position
                    current_azimuth, current_elevation, stepper_steps = move_to_polar_position(
                        pi, current_azimuth + 20, current_elevation, stepper_steps
                    )
                    time.sleep(1)
                    
            elif current_state == "TRACKING":
                print("\n=== STEP 3: Tracking Mode ===")
                print(f"Tracking target around last detected point: {last_detected_point}")
                
                # Extract coordinates from last detected point
                last_distance, last_azimuth, last_elevation, last_timestamp = last_detected_point
                
                # Scan around the last detected point
                scan_radius = 5  # degrees
                scan_positions = [
                    (0, 0),  # Center
                    (-scan_radius, 0), (scan_radius, 0),  # Left/right
                    (0, -scan_radius), (0, scan_radius),  # Up/down
                    (-scan_radius, -scan_radius), (scan_radius, scan_radius),  # Diagonals
                ]
                
                new_detection = None
                
                for az_offset, el_offset in scan_positions:
                    scan_azimuth = last_azimuth + az_offset
                    scan_elevation = last_elevation + el_offset
                    
                    # Clamp elevation to servo limits
                    scan_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, scan_elevation))
                    
                    print(f"Scanning at: Az={scan_azimuth:.1f}°, El={scan_elevation:.1f}°")
                    
                    # Move to scan position
                    current_azimuth, current_elevation, stepper_steps = move_to_polar_position(
                        pi, scan_azimuth, scan_elevation, stepper_steps
                    )
                    
                    # Wait for motors to settle
                    time.sleep(0.5)
                    
                    # Check for target at this position
                    try:
                        distance = lidar_data_queue.get(timeout=1.0)
                        reference = get_reference_distance(current_azimuth, current_elevation, calibration_data)
                        
                        if is_anomaly(distance, reference, ANOMALY_FACTOR):
                            print(f"New target detected: {distance:.1f}cm at ({current_azimuth:.1f}°, {current_elevation:.1f}°)")
                            new_detection = [distance, current_azimuth, current_elevation, time.time()]
                            break
                            
                    except queue.Empty:
                        continue
                
                if new_detection:
                    # Update last detected point and continue tracking
                    last_detected_point = new_detection
                    print(f"Updated tracking point: {new_detection}")
                    # Continue in tracking state
                else:
                    print("No new target found, returning to search mode")
                    current_state = "SEARCHING"
                
                # Small delay before next tracking cycle
                time.sleep(0.5)
                
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Cleaning up...")
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        reset_stepper_pos(stepper_steps)
        pi.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()