# tracking_functions.py
import time
import queue
from constants import *
from anomaly_check import get_interpolated_reference_distance

def detect_at_position(pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                      max_samples=50, detection_threshold_factor=ANOMALY_FACTOR):
    """
    Stay at current position and collect LiDAR samples to detect if target is present.
    
    Args:
        pi: pigpio instance 
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees
        current_elevation: Current elevation position in degrees
        max_samples: Maximum number of LiDAR samples to collect
        detection_threshold_factor: Factor to determine if reading is anomalous
        
    Returns:
        tuple: (detection_found, average_distance, sample_count)
            - detection_found: Boolean indicating if target was detected
            - average_distance: Average distance of detected readings (or None)
            - sample_count: Number of valid samples collected
    """
    
    print(f"Searching at position: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°")
    
    # Clear any old readings from queue
    while not lidar_data_queue.empty():
        try:
            lidar_data_queue.get_nowait()
        except queue.Empty:
            break
    
    # Get reference distance for this position
    reference_distance = get_interpolated_reference_distance(current_azimuth, current_elevation, calibration_data)
    detection_threshold = reference_distance * detection_threshold_factor
    
    print(f"Reference distance: {reference_distance:.1f}cm, Detection threshold: {detection_threshold:.1f}cm")
    
    # Collect samples at this position
    valid_readings = []
    anomaly_readings = []
    sample_count = 0
    start_time = time.time()
    max_wait_time = 2.0  # Maximum time to wait for samples (seconds)
    
    while sample_count < max_samples and (time.time() - start_time) < max_wait_time:
        try:
            # Get LiDAR reading with short timeout
            distance = lidar_data_queue.get(timeout=0.1)
            
            if distance > 0 and distance < SENSOR_MAX:  # Valid reading
                valid_readings.append(distance)
                sample_count += 1
                
                # Check if this reading indicates target presence
                if distance < detection_threshold:
                    anomaly_readings.append(distance)
                    print(f"  Sample {sample_count}: {distance:.1f}cm - ANOMALY DETECTED!")
                else:
                    print(f"  Sample {sample_count}: {distance:.1f}cm - normal")
                
                # Early detection: if we have enough anomaly readings, we can conclude
                if len(anomaly_readings) >= 3:
                    print(f"Target detected with {len(anomaly_readings)} anomalous readings!")
                    avg_distance = sum(anomaly_readings) / len(anomaly_readings)
                    return True, avg_distance, sample_count
                    
        except queue.Empty:
            # No reading available, continue waiting
            time.sleep(0.05)
            continue
        except Exception as e:
            print(f"Error reading LiDAR: {e}")
            continue
    
    # Analysis of collected samples
    if not valid_readings:
        print("No valid LiDAR readings collected")
        return False, None, 0
    
    # Determine if target is present based on anomaly ratio
    anomaly_ratio = len(anomaly_readings) / len(valid_readings) if valid_readings else 0
    detection_found = anomaly_ratio >= 0.3  # Need at least 30% anomalous readings
    
    if detection_found:
        avg_distance = sum(anomaly_readings) / len(anomaly_readings)
        print(f"Target detected! {len(anomaly_readings)}/{len(valid_readings)} readings were anomalous")
        print(f"Average target distance: {avg_distance:.1f}cm")
        return True, avg_distance, sample_count
    else:
        print(f"No target detected. {len(anomaly_readings)}/{len(valid_readings)} readings were anomalous (need ≥30%)")
        return False, None, sample_count


def perform_local_search(pi, lidar_data_queue, calibration_data, center_azimuth, center_elevation, 
                        stepper_steps, search_radius_deg=2.0):
    """
    Perform a small local search around the predicted position to account for prediction errors.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings  
        calibration_data: Reference calibration data
        center_azimuth: Center azimuth position for search
        center_elevation: Center elevation position for search
        stepper_steps: Current stepper step count
        search_radius_deg: Radius in degrees to search around center position
        
    Returns:
        tuple: (detection_found, best_measurement, final_azimuth, final_elevation, final_stepper_steps)
    """
    
    print(f"Performing local search around Az={center_azimuth:.1f}°, El={center_elevation:.1f}°")
    print(f"Search radius: ±{search_radius_deg:.1f}°")
    
    from move_motors import move_to_polar_position
    
    # Define search pattern (spiral outward from center)
    search_positions = [
        (0, 0),  # Center position first
        (-search_radius_deg, 0), (search_radius_deg, 0),  # Left/right
        (0, -search_radius_deg), (0, search_radius_deg),  # Down/up  
        (-search_radius_deg, -search_radius_deg), (search_radius_deg, search_radius_deg),  # Diagonals
        (search_radius_deg, -search_radius_deg), (-search_radius_deg, search_radius_deg)
    ]
    
    best_detection = None
    best_measurement = None
    
    for i, (az_offset, el_offset) in enumerate(search_positions):
        search_azimuth = center_azimuth + az_offset
        search_elevation = center_elevation + el_offset
        
        # Clamp elevation to servo limits
        search_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, search_elevation))
        
        print(f"\nSearch position {i+1}/{len(search_positions)}: Az={search_azimuth:.1f}°, El={search_elevation:.1f}°")
        
        # Move to search position
        actual_az, actual_el, stepper_steps = move_to_polar_position(pi, search_azimuth, search_elevation, stepper_steps)
        
        # Wait for motors to settle
        time.sleep(0.3)
        
        # Search for target at this position
        detection_found, avg_distance, sample_count = detect_at_position(
            pi, lidar_data_queue, calibration_data, actual_az, actual_el
        )
        
        if detection_found:
            measurement = [avg_distance, actual_az, actual_el, time.time()]
            
            # Use first detection found, or keep the one with shortest distance (closest target)
            if best_detection is None or avg_distance < best_measurement[0]:
                best_detection = True
                best_measurement = measurement
                print(f"*** BEST DETECTION SO FAR: {avg_distance:.1f}cm at ({actual_az:.1f}°, {actual_el:.1f}°) ***")
            
            # For efficiency, you could break here if you only want the first detection
            # break
    
    # Return to center position if no detection found
    if best_detection is None:
        print("No target found in local search, returning to center position")
        actual_az, actual_el, stepper_steps = move_to_polar_position(pi, center_azimuth, center_elevation, stepper_steps)
        return False, None, actual_az, actual_el, stepper_steps
    else:
        # Move to best detection position
        best_az, best_el = best_measurement[1], best_measurement[2]
        actual_az, actual_el, stepper_steps = move_to_polar_position(pi, best_az, best_el, stepper_steps)
        print(f"Moving to best detection position: Az={best_az:.1f}°, El={best_el:.1f}°")
        return True, best_measurement, actual_az, actual_el, stepper_steps


# Modified tracking function for main.py
def perform_tracking_detection(pi, lidar_data_queue, calibration_data, predicted_azimuth, predicted_elevation, stepper_steps):
    """
    Perform target detection at predicted location for Kalman tracking.
    This replaces the scanning sequence during tracking phase.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data
        predicted_azimuth: Predicted azimuth from Kalman filter
        predicted_elevation: Predicted elevation from Kalman filter  
        stepper_steps: Current stepper step count
        
    Returns:
        tuple: (new_measurement, final_azimuth, final_elevation, final_stepper_steps)
            - new_measurement: [distance, azimuth, elevation, timestamp] if found, None otherwise
            - final_azimuth: Final azimuth position
            - final_elevation: Final elevation position
            - final_stepper_steps: Final stepper step count
    """
    
    from move_motors import move_to_polar_position
    
    print(f"=== TRACKING DETECTION ===")
    print(f"Moving to predicted position: Az={predicted_azimuth:.1f}°, El={predicted_elevation:.1f}°")
    
    # Move to predicted position
    actual_azimuth, actual_elevation, stepper_steps = move_to_polar_position(
        pi, predicted_azimuth, predicted_elevation, stepper_steps
    )
    
    # Wait for motors to settle
    time.sleep(0.5)
    
    # Try detection at exact predicted position first
    detection_found, avg_distance, sample_count = detect_at_position(
        pi, lidar_data_queue, calibration_data, actual_azimuth, actual_elevation
    )
    
    if detection_found:
        print(f"Target found at predicted position!")
        measurement = [avg_distance, actual_azimuth, actual_elevation, time.time()]
        return measurement, actual_azimuth, actual_elevation, stepper_steps
    
    # If not found at exact position, perform local search
    print("Target not found at exact predicted position, performing local search...")
    
    detection_found, best_measurement, final_az, final_el, stepper_steps = perform_local_search(
        pi, lidar_data_queue, calibration_data, actual_azimuth, actual_elevation, stepper_steps
    )
    
    if detection_found:
        print(f"Target found during local search!")
        return best_measurement, final_az, final_el, stepper_steps
    else:
        print("Target not found in local search area")
        return None, final_az, final_el, stepper_steps