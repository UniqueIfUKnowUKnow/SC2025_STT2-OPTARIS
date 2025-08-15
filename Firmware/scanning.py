# scanning.py
import RPi.GPIO as GPIO
import time
import queue
from constants import *
from move_motors import stepper_step, move_to_polar_position
from anomaly_check import get_interpolated_reference_distance

def perform_sweep(pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                 stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, 
                 detections_required, direction="forward", ):
    """
    Perform a single sweep (forward or reverse) and detect anomalies.
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees
        current_elevation: Current elevation position in degrees
        stepper_steps: Current stepper motor step count
        anomaly_locations: List to store detected anomalies
        anomaly_averaged_coords: List to store averaged anomaly coordinates
        anomaly_count: Current count of anomaly groups detected
        direction: "forward" or "reverse" sweep direction
        
    Returns:
        tuple: (updated_azimuth, updated_stepper_steps, updated_anomaly_count, state_change_needed)
            - updated_azimuth: New azimuth position after sweep
            - updated_stepper_steps: New stepper step count
            - updated_anomaly_count: Updated anomaly count
            - state_change_needed: Boolean indicating if state should change to "DETECTED"
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
            reference = get_interpolated_reference_distance(current_azimuth, current_elevation, calibration_data)
            
            print(distance - (reference * ANOMALY_FACTOR))
            
            # Check for anomaly
            if distance < reference * ANOMALY_FACTOR:
                # Store as [distance, azimuth, elevation] triplet
                
                anomaly_locations.append([distance, current_azimuth, current_elevation, time.time()])
                print(f"Anomaly detected: {distance:.1f}cm at ({current_azimuth:.1f}°, {current_elevation:.1f}°), "
                      f"expected: {reference:.1f}cm, difference: {distance - (reference * ANOMALY_FACTOR):.1f}cm")
                
            # Check if we have enough anomalies to declare detection
            if len(anomaly_locations) >= 3:
                anomaly_averaged_coords.append([tuple(round(sum(col) / len(col), 2) for col in zip(*anomaly_locations))])
                anomaly_locations.clear()  # Clear the list for next group    
                anomaly_count += 1
                if detections_required > 1:
                    current_azimuth, current_elevation, stepper_steps = move_to_polar_position(
                        pi, current_azimuth + SWEEP_RANGE, current_elevation, stepper_steps)
            
        except queue.Empty:
            continue
        
        # Check if we should change state during the sweep
        if anomaly_count >= INITIAL_SWEEP_DETECTIONS_COUNT:
            return current_azimuth, stepper_steps, anomaly_count, True
    
    return current_azimuth, stepper_steps, anomaly_count, False


def perform_scanning_sequence(pi, lidar_data_queue, calibration_data, current_azimuth, 
                            current_elevation, stepper_steps, anomaly_locations, 
                            anomaly_averaged_coords, anomaly_count, detections_required):
    """
    Perform a complete scanning sequence (forward and reverse sweeps).
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees
        current_elevation: Current elevation position in degrees
        stepper_steps: Current stepper motor step count
        anomaly_locations: List to store detected anomalies
        anomaly_averaged_coords: List to store averaged anomaly coordinates
        anomaly_count: Current count of anomaly groups detected
        
    Returns:
        tuple: (updated_azimuth, updated_elevation, updated_stepper_steps, updated_anomaly_count, state_change_needed)
    """
    
    print("Scanning area...")
    
    # Clear LiDAR queue before starting
    while not lidar_data_queue.empty():
        try:
            lidar_data_queue.get_nowait()
        except queue.Empty:
            break
    
    # Perform forward sweep
    current_azimuth, stepper_steps, anomaly_count, state_change = perform_sweep(
        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation,
        stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, detections_required , "forward"
    )
    
    # Check if we should change state after forward sweep
    if state_change:
        return current_azimuth, current_elevation, stepper_steps, anomaly_count, True
    
    # Perform reverse sweep
    current_azimuth, stepper_steps, anomaly_count, state_change = perform_sweep(
        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation,
        stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, detections_required, "reverse" 
    )
    
    # Final check for state change
    if anomaly_count >= detections_required:
        state_change = True
    
    return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, state_change