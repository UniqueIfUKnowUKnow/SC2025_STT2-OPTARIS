# scanning.py
import RPi.GPIO as GPIO
import time
import queue
from constants import *
from move_motors import stepper_step, move_to_polar_position, set_servo_angle
from anomaly_check import get_interpolated_reference_distance

def perform_sweep(pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                 stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, 
                 detections_required, azimuth_range, direction="forward"):
    
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
        detections_required: Number of detections needed to trigger state change
        azimuth_range: Azimuth sweep range in degrees
        direction: "forward" or "reverse" sweep direction
        
    Returns:
        tuple: (updated_azimuth, updated_stepper_steps, updated_anomaly_count, state_change_needed)
    """
    
    # Set motor direction
    if direction == "forward":
        GPIO.output(DIR_PIN, GPIO.HIGH)
        print(f"Forward sweep (range: {azimuth_range}°)...")
        step_increment = 1
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)
        print(f"Reverse sweep (range: {azimuth_range}°)...")
        step_increment = -1
    
    # Calculate number of steps for the sweep based on input range
    steps_to_take = round(STEPS_PER_REVOLUTION * azimuth_range / 360)
    
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
                if anomaly_count < detections_required and detections_required > 1:
                    current_azimuth, current_elevation, stepper_steps = move_to_polar_position(
                        pi, current_azimuth + AZIMUTH_AMOUNT, current_elevation + TILT_AMOUNT, stepper_steps)
                    
            print(anomaly_count, detections_required)
        except queue.Empty:
            continue
        
        # Check if we should change state during the sweep
        if anomaly_count >= detections_required:
            return current_azimuth, stepper_steps, anomaly_count, True
    
    return current_azimuth, stepper_steps, anomaly_count, False


def perform_scanning_sequence(pi, lidar_data_queue, calibration_data, current_azimuth, 
                            current_elevation, stepper_steps, anomaly_locations, 
                            anomaly_averaged_coords, anomaly_count, detections_required,
                            azimuth_range=SWEEP_RANGE, elevation_range=5):
    """
    Perform a complete scanning sequence with configurable azimuth and elevation ranges.
    The servo will move up and down within the elevation range while stepper sweeps left/right.
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees
        current_elevation: Current elevation position in degrees (center of elevation scan)
        stepper_steps: Current stepper motor step count
        anomaly_locations: List to store detected anomalies
        anomaly_averaged_coords: List to store averaged anomaly coordinates
        anomaly_count: Current count of anomaly groups detected
        detections_required: Number of detections needed to trigger state change
        azimuth_range: Azimuth sweep range in degrees (default: SWEEP_RANGE from constants)
        elevation_range: Elevation sweep range in degrees (±range from center, default: 5)
        
    Returns:
        tuple: (updated_azimuth, updated_elevation, updated_stepper_steps, updated_anomaly_coords, updated_anomaly_count, state_change_needed)
    """
    
    print(f"Scanning area with Az range: ±{azimuth_range/2:.1f}°, El range: ±{elevation_range:.1f}°")
    print(f"Center position: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°")
    
    # Clear LiDAR queue before starting
    while not lidar_data_queue.empty():
        try:
            lidar_data_queue.get_nowait()
        except queue.Empty:
            break
    
    # Calculate elevation limits
    center_elevation = current_elevation
    min_elevation = max(SERVO_SWEEP_START, center_elevation - elevation_range)
    max_elevation = min(SERVO_SWEEP_END, center_elevation + elevation_range)
    
    print(f"Elevation scan limits: {min_elevation:.1f}° to {max_elevation:.1f}°")
    
    # Start at center elevation
    current_scan_elevation = center_elevation
    set_servo_angle(pi, current_scan_elevation)
    time.sleep(0.01)  # Allow servo to settle
    
    # Scanning state variables
    elevation_direction = 1  # 1 for up, -1 for down
    sweep_direction = "forward"  # Start with forward sweep
    sweep_count = 0
    
    # Continue scanning until we get enough detections or reach a reasonable limit
    max_sweeps = (max_elevation - min_elevation + 1) * 4  # Safety limit
    
    while anomaly_count < detections_required and sweep_count < max_sweeps:
        print(f"\n--- Sweep {sweep_count + 1} at elevation {current_scan_elevation:.1f}° ---")
        
        # Perform sweep at current elevation
        current_azimuth, stepper_steps, anomaly_count, state_change = perform_sweep(
            pi, lidar_data_queue, calibration_data, current_azimuth, current_scan_elevation,
            stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, 
            detections_required, azimuth_range, sweep_direction
        )
        
        # Check if we should change state after this sweep
        if state_change:
            print(f"State change triggered after sweep {sweep_count + 1}")
            return current_azimuth, current_scan_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, True
        
        # Update elevation for next sweep
        next_elevation = current_scan_elevation + elevation_direction
        
        # Check if we need to reverse elevation direction
        if next_elevation > max_elevation:
            elevation_direction = -1
            next_elevation = current_scan_elevation + elevation_direction
        elif next_elevation < min_elevation:
            elevation_direction = 1
            next_elevation = current_scan_elevation + elevation_direction
        
        # Move to next elevation if within bounds
        if min_elevation <= next_elevation <= max_elevation:
            current_scan_elevation = next_elevation
            set_servo_angle(pi, current_scan_elevation)
            time.sleep(0.01)  # Allow servo to settle
            print(f"Moved to elevation: {current_scan_elevation:.1f}°")
        
        # Alternate sweep direction for next sweep
        sweep_direction = "reverse" if sweep_direction == "forward" else "forward"
        sweep_count += 1
        
    
    # Return to center elevation when done
    if current_scan_elevation != center_elevation:
        print(f"Returning to center elevation: {center_elevation:.1f}°")
        set_servo_angle(pi, center_elevation)
        time.sleep(0.01)
        current_elevation = center_elevation
    else:
        current_elevation = current_scan_elevation
    
    # Final check for state change
    state_change_needed = anomaly_count >= detections_required
    
    if state_change_needed:
        print(f"Scanning complete: {anomaly_count}/{detections_required} detections found")
    else:
        print(f"Scanning finished: {anomaly_count}/{detections_required} detections found (insufficient for state change)")
    
    return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, state_change_needed


def perform_targeted_scan(pi, lidar_data_queue, calibration_data, target_azimuth, target_elevation,
                         stepper_steps, azimuth_range=5, elevation_range=3):
    """
    Perform a targeted scan around a specific location (useful for tracking mode).
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        target_azimuth: Target azimuth position in degrees
        target_elevation: Target elevation position in degrees
        stepper_steps: Current stepper motor step count
        azimuth_range: Azimuth search range in degrees (default: 5)
        elevation_range: Elevation search range in degrees (default: 3)
        
    Returns:
        tuple: (detection_found, best_detection, final_azimuth, final_elevation, final_stepper_steps)
            - detection_found: Boolean indicating if target was found
            - best_detection: [distance, azimuth, elevation, timestamp] of best detection
            - final_azimuth: Final azimuth position
            - final_elevation: Final elevation position  
            - final_stepper_steps: Final stepper step count
    """
    
    print(f"Targeted scan around Az={target_azimuth:.1f}°, El={target_elevation:.1f}°")
    print(f"Search range: Az±{azimuth_range/2:.1f}°, El±{elevation_range:.1f}°")
    
    # Move to target position first
    current_azimuth, current_elevation, stepper_steps = move_to_polar_position(
        pi, target_azimuth, target_elevation, stepper_steps
    )
    
    # Initialize scanning variables for targeted search
    anomaly_locations = []
    anomaly_averaged_coords = []
    anomaly_count = 0
    
    # Perform scan with smaller ranges
    final_azimuth, final_elevation, final_stepper_steps, found_coords, final_count, found = perform_scanning_sequence(
        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation,
        stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count,
        detections_required=1, azimuth_range=azimuth_range, elevation_range=elevation_range
    )
    
    if found and found_coords:
        # Return the best detection (first one found)
        best_detection = list(found_coords[0][0])  # Convert from tuple
        return True, best_detection, final_azimuth, final_elevation, final_stepper_steps
    else:
        return False, None, final_azimuth, final_elevation, final_stepper_steps