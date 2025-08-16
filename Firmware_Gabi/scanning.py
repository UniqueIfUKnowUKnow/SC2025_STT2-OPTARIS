# scanning.py
# Simplified scanning for drone detection
import RPi.GPIO as GPIO
import time
import queue
from constants import *
from move_motors import stepper_step, move_to_polar_position
from anomaly_detection import get_reference_distance, is_anomaly

def perform_sweep(pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                 stepper_steps, direction="forward"):
    """
    Perform a single sweep (forward or reverse) and detect anomalies.
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees
        current_elevation: Current elevation position in degrees
        stepper_steps: Current stepper motor step count
        direction: "forward" or "reverse" sweep direction
        
    Returns:
        tuple: (updated_azimuth, updated_stepper_steps, anomaly_detected, anomaly_location)
    """
    
    # Set motor direction
    if direction == "forward":
        GPIO.output(DIR_PIN, GPIO.HIGH)
        print("Forward sweep...")
        step_increment = 1
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)
        print("Reverse sweep...")
        step_increment = -1
    
    # Calculate number of steps for the sweep
    steps_to_take = round(STEPS_PER_REVOLUTION * SWEEP_RANGE / 360)
    
    for i in range(steps_to_take):
        # Step the motor
        stepper_step()
        stepper_steps += step_increment
        time.sleep(0.0005)

        # Calculate current azimuth position
        current_azimuth = (stepper_steps / STEPS_PER_REVOLUTION) * 360
        
        # Get LiDAR reading and process
        try:
            distance = lidar_data_queue.get_nowait()
            
            # Get reference distance for this position
            reference = get_reference_distance(current_azimuth, current_elevation, calibration_data)
            
            # Check for anomaly (potential drone)
            if is_anomaly(distance, reference, ANOMALY_FACTOR):
                print(f"Anomaly detected: {distance:.1f}cm at ({current_azimuth:.1f}°, {current_elevation:.1f}°)")
                return current_azimuth, stepper_steps, True, [distance, current_azimuth, current_elevation, time.time()]
                
        except queue.Empty:
            continue
    
    return current_azimuth, stepper_steps, False, None

def perform_scanning_sequence(pi, lidar_data_queue, calibration_data, current_azimuth, 
                            current_elevation, stepper_steps):
    """
    Perform a complete scanning sequence (forward and reverse sweeps).
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees
        current_elevation: Current elevation position in degrees
        stepper_steps: Current stepper motor step count
        
    Returns:
        tuple: (updated_azimuth, updated_elevation, updated_stepper_steps, anomaly_detected, anomaly_location)
    """
    
    print("Scanning area for drones...")
    
    # Clear LiDAR queue before starting
    while not lidar_data_queue.empty():
        try:
            lidar_data_queue.get_nowait()
        except queue.Empty:
            break
    
    # Perform forward sweep
    current_azimuth, stepper_steps, anomaly_detected, anomaly_location = perform_sweep(
        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation,
        stepper_steps, "forward"
    )
    
    # Check if anomaly detected during forward sweep
    if anomaly_detected:
        return current_azimuth, current_elevation, stepper_steps, True, anomaly_location
    
    # Perform reverse sweep
    current_azimuth, stepper_steps, anomaly_detected, anomaly_location = perform_sweep(
        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation,
        stepper_steps, "reverse" 
    )
    
    return current_azimuth, current_elevation, stepper_steps, anomaly_detected, anomaly_location