# enhanced_scanning.py
import RPi.GPIO as GPIO
import time
import queue
import numpy as np
from constants import *
from move_motors import stepper_step, move_to_polar_position, set_servo_angle
from anomaly_check import get_interpolated_reference_distance

def perform_point_to_point_sweep(pi, lidar_data_queue, calibration_data, start_azimuth, start_elevation,
                                end_azimuth, end_elevation, stepper_steps, anomaly_locations, 
                                anomaly_averaged_coords, anomaly_count, detections_required, 
                                num_steps=50, direction="forward"):
    """
    Perform a sweep between any two arbitrary points with smooth motion.
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        start_azimuth: Starting azimuth position in degrees
        start_elevation: Starting elevation position in degrees
        end_azimuth: Ending azimuth position in degrees
        end_elevation: Ending elevation position in degrees
        stepper_steps: Current stepper motor step count
        anomaly_locations: List to store detected anomalies
        anomaly_averaged_coords: List to store averaged anomaly coordinates
        anomaly_count: Current count of anomaly groups detected
        detections_required: Number of detections needed to trigger state change
        num_steps: Number of interpolation steps between start and end points
        direction: "forward" or "reverse" (affects which point is start vs end)
        
    Returns:
        tuple: (final_azimuth, final_elevation, final_stepper_steps, updated_anomaly_count, state_change_needed)
    """
    
    # Handle direction by swapping start/end if reverse
    if direction == "reverse":
        start_azimuth, end_azimuth = end_azimuth, start_azimuth
        start_elevation, end_elevation = end_elevation, start_elevation
    
    print(f"{direction.capitalize()} sweep from ({start_azimuth:.1f}°, {start_elevation:.1f}°) to ({end_azimuth:.1f}°, {end_elevation:.1f}°)")
    
    # Calculate the sweep path using linear interpolation
    # Handle azimuth wrap-around for shortest path
    azimuth_diff = end_azimuth - start_azimuth
    if azimuth_diff > 180:
        azimuth_diff -= 360
    elif azimuth_diff < -180:
        azimuth_diff += 360
    
    # Create smooth interpolation between points
    t_values = np.linspace(0, 1, num_steps)
    azimuth_path = start_azimuth + azimuth_diff * t_values
    elevation_path = start_elevation + (end_elevation - start_elevation) * t_values
    
    # Normalize azimuth values to 0-360 range
    azimuth_path = azimuth_path % 360
    
    print(f"Sweep path calculated: {num_steps} steps, azimuth range: {azimuth_diff:.1f}°, elevation range: {end_elevation - start_elevation:.1f}°")
    
    current_azimuth = start_azimuth
    current_elevation = start_elevation
    
    # Move through each point in the sweep path
    for i, (target_az, target_el) in enumerate(zip(azimuth_path, elevation_path)):
        # Ensure elevation is within servo limits
        target_el = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, target_el))
        
        # Move to next position
        current_azimuth, current_elevation, stepper_steps = move_to_polar_position(
            pi, target_az, target_el, stepper_steps
        )
        
        # Small delay for smooth motion and LiDAR reading
        time.sleep(0.05)  # Adjust for desired sweep speed
        
        # Get LiDAR reading and process
        try:
            distance = lidar_data_queue.get_nowait()
            
            # Get reference distance for this position
            reference = get_interpolated_reference_distance(current_azimuth, current_elevation, calibration_data)
            
            difference = distance - (reference * ANOMALY_FACTOR)
            print(f"Step {i+1}/{num_steps}: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°, Dist={distance:.1f}cm, Diff={difference:.1f}cm")
            
            # Check for anomaly
            if distance < reference * ANOMALY_FACTOR:
                # Store as [distance, azimuth, elevation, timestamp] quadruplet
                anomaly_locations.append([distance, current_azimuth, current_elevation, time.time()])
                print(f"  *** ANOMALY DETECTED: {distance:.1f}cm at ({current_azimuth:.1f}°, {current_elevation:.1f}°), expected: {reference:.1f}cm ***")
                
            # Check if we have enough anomalies to declare detection
            if len(anomaly_locations) >= 3:
                # Calculate average of the anomaly group
                avg_anomaly = [sum(col) / len(col) for col in zip(*anomaly_locations)]
                avg_anomaly = [round(val, 2) for val in avg_anomaly]
                anomaly_averaged_coords.append([tuple(avg_anomaly)])
                
                anomaly_locations.clear()  # Clear the list for next group    
                anomaly_count += 1
                
                print(f"Anomaly group {anomaly_count} detected: {avg_anomaly}")
                
                # Optional: Move to next search area if more detections needed
                if anomaly_count < detections_required and detections_required > 1:
                    offset_az = current_azimuth + AZIMUTH_AMOUNT
                    offset_el = current_elevation + TILT_AMOUNT
                    current_azimuth, current_elevation, stepper_steps = move_to_polar_position(
                        pi, offset_az, offset_el, stepper_steps)
                    
            # Check if we should change state during the sweep
            if anomaly_count >= detections_required:
                print(f"Detection threshold reached: {anomaly_count}/{detections_required}")
                return current_azimuth, current_elevation, stepper_steps, anomaly_count, True
                
        except queue.Empty:
            continue
        except Exception as e:
            print(f"Error during sweep step {i}: {e}")
            continue
    
    print(f"Sweep completed. Final position: ({current_azimuth:.1f}°, {current_elevation:.1f}°)")
    return current_azimuth, current_elevation, stepper_steps, anomaly_count, False


def perform_arbitrary_scanning_sequence(pi, lidar_data_queue, calibration_data, current_azimuth, 
                                       current_elevation, stepper_steps, anomaly_locations, 
                                       anomaly_averaged_coords, anomaly_count, detections_required,
                                       sweep_points=None, num_steps_per_sweep=50):
    """
    Perform scanning between arbitrary points defined by sweep_points.
    If no sweep_points provided, falls back to traditional rectangular scanning.
    
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
        sweep_points: List of (azimuth, elevation) tuples defining sweep endpoints
                     If None, creates default rectangular pattern
        num_steps_per_sweep: Number of interpolation steps for each sweep
        
    Returns:
        tuple: (updated_azimuth, updated_elevation, updated_stepper_steps, updated_anomaly_coords, updated_anomaly_count, state_change_needed)
    """
    
    print(f"Starting arbitrary scanning sequence from ({current_azimuth:.1f}°, {current_elevation:.1f}°)")
    
    # Clear LiDAR queue before starting
    while not lidar_data_queue.empty():
        try:
            lidar_data_queue.get_nowait()
        except queue.Empty:
            break
    
    # Default sweep pattern if none provided (rectangular grid)
    if sweep_points is None:
        azimuth_range = SWEEP_RANGE
        elevation_range = 5
        
        # Create default rectangular sweep points
        center_az = current_azimuth
        center_el = current_elevation
        
        sweep_points = [
            # Horizontal sweeps at different elevations
            ((center_az - azimuth_range/2, center_el - elevation_range/2), (center_az + azimuth_range/2, center_el - elevation_range/2)),
            ((center_az + azimuth_range/2, center_el), (center_az - azimuth_range/2, center_el)),
            ((center_az - azimuth_range/2, center_el + elevation_range/2), (center_az + azimuth_range/2, center_el + elevation_range/2)),
        ]
        print(f"Using default rectangular sweep pattern: ±{azimuth_range/2:.1f}° azimuth, ±{elevation_range/2:.1f}° elevation")
    
    # Validate sweep points format
    validated_points = []
    for i, point_pair in enumerate(sweep_points):
        if len(point_pair) != 2:
            print(f"Warning: Sweep point pair {i} invalid format, skipping")
            continue
        
        start_point, end_point = point_pair
        if len(start_point) != 2 or len(end_point) != 2:
            print(f"Warning: Sweep points {i} should be (azimuth, elevation) tuples, skipping")
            continue
            
        start_az, start_el = start_point
        end_az, end_el = end_point
        
        # Clamp elevations to servo limits
        start_el = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, start_el))
        end_el = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, end_el))
        
        validated_points.append(((start_az, start_el), (end_az, end_el)))
    
    if not validated_points:
        print("Error: No valid sweep points provided")
        return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, False
    
    print(f"Validated {len(validated_points)} sweep segments")
    
    # Execute sweeps
    sweep_count = 0
    max_sweeps = len(validated_points) * 2  # Forward and reverse for each segment
    
    for sweep_idx, ((start_az, start_el), (end_az, end_el)) in enumerate(validated_points):
        if anomaly_count >= detections_required:
            break
            
        print(f"\n=== Sweep Segment {sweep_idx + 1}/{len(validated_points)} ===")
        
        # Forward sweep
        if anomaly_count < detections_required:
            print(f"Forward sweep {sweep_count + 1}")
            current_azimuth, current_elevation, stepper_steps, anomaly_count, state_change = perform_point_to_point_sweep(
                pi, lidar_data_queue, calibration_data, start_az, start_el, end_az, end_el,
                stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, 
                detections_required, num_steps_per_sweep, "forward"
            )
            sweep_count += 1
            
            if state_change:
                print(f"State change triggered during forward sweep {sweep_count}")
                return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, True
        
        # Reverse sweep
        if anomaly_count < detections_required:
            print(f"Reverse sweep {sweep_count + 1}")
            current_azimuth, current_elevation, stepper_steps, anomaly_count, state_change = perform_point_to_point_sweep(
                pi, lidar_data_queue, calibration_data, start_az, start_el, end_az, end_el,
                stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, 
                detections_required, num_steps_per_sweep, "reverse"
            )
            sweep_count += 1
            
            if state_change:
                print(f"State change triggered during reverse sweep {sweep_count}")
                return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, True
    
    # Final check for state change
    state_change_needed = anomaly_count >= detections_required
    
    if state_change_needed:
        print(f"Scanning complete: {anomaly_count}/{detections_required} detections found")
    else:
        print(f"Scanning finished: {anomaly_count}/{detections_required} detections found (insufficient for state change)")
    
    return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, state_change_needed


def create_diagonal_sweep_points(center_azimuth, center_elevation, azimuth_range=10, elevation_range=5, num_sweeps=4):
    """
    Helper function to create diagonal sweep patterns.
    
    Args:
        center_azimuth: Center azimuth position
        center_elevation: Center elevation position
        azimuth_range: Total azimuth range to cover
        elevation_range: Total elevation range to cover
        num_sweeps: Number of diagonal sweeps to create
        
    Returns:
        List of sweep point pairs for diagonal scanning
    """
    
    half_az = azimuth_range / 2
    half_el = elevation_range / 2
    
    diagonal_sweeps = [
        # Main diagonals
        ((center_azimuth - half_az, center_elevation - half_el), (center_azimuth + half_az, center_elevation + half_el)),
        ((center_azimuth + half_az, center_elevation - half_el), (center_azimuth - half_az, center_elevation + half_el)),
        
        # Additional cross patterns if requested
        ((center_azimuth - half_az, center_elevation), (center_azimuth + half_az, center_elevation)),
        ((center_azimuth, center_elevation - half_el), (center_azimuth, center_elevation + half_el)),
    ]
    
    return diagonal_sweeps[:num_sweeps]


def create_radial_sweep_points(center_azimuth, center_elevation, radius_deg=5, num_rays=8):
    """
    Helper function to create radial sweep patterns (star pattern).
    
    Args:
        center_azimuth: Center azimuth position
        center_elevation: Center elevation position
        radius_deg: Radius of the sweep pattern in degrees
        num_rays: Number of radial sweeps
        
    Returns:
        List of sweep point pairs for radial scanning
    """
    
    radial_sweeps = []
    
    for i in range(num_rays):
        angle = 2 * np.pi * i / num_rays
        
        # Calculate end point for this ray
        end_azimuth = center_azimuth + radius_deg * np.cos(angle)
        end_elevation = center_elevation + radius_deg * np.sin(angle)
        
        # Clamp elevation to valid range
        end_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, end_elevation))
        
        radial_sweeps.append(((center_azimuth, center_elevation), (end_azimuth, end_elevation)))
    
    return radial_sweeps


# Example usage functions that can replace existing scanning calls
def perform_diagonal_scanning(pi, lidar_data_queue, calibration_data, current_azimuth, 
                            current_elevation, stepper_steps, anomaly_locations, 
                            anomaly_averaged_coords, anomaly_count, detections_required,
                            azimuth_range=10, elevation_range=5):
    """
    Convenience function for diagonal scanning pattern.
    """
    diagonal_points = create_diagonal_sweep_points(current_azimuth, current_elevation, azimuth_range, elevation_range)
    
    return perform_arbitrary_scanning_sequence(
        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation,
        stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count,
        detections_required, sweep_points=diagonal_points
    )


def perform_radial_scanning(pi, lidar_data_queue, calibration_data, current_azimuth, 
                          current_elevation, stepper_steps, anomaly_locations, 
                          anomaly_averaged_coords, anomaly_count, detections_required,
                          radius_deg=5, num_rays=6):
    """
    Convenience function for radial (star) scanning pattern.
    """
    radial_points = create_radial_sweep_points(current_azimuth, current_elevation, radius_deg, num_rays)
    
    return perform_arbitrary_scanning_sequence(
        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation,
        stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count,
        detections_required, sweep_points=radial_points
    )