import RPi.GPIO as GPIO
import time
import queue
from constants import *
from move_motors import set_servo_angle, stepper_step

def sweep_scan_for_anomaly(pi, lidar_data_queue, calibration_data, 
                            current_azimuth_steps, current_elevation,
                            azimuth_start, azimuth_end,
                            elevation_start, elevation_end,
                            anomaly_threshold_factor=0.7, 
                            consecutive_detections_needed=3,
                            max_distance_change=50):
    """
    Sweeps the stepper motor left and right within specified ranges, comparing
    LiDAR readings to calibration reference data to detect anomalies.
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR distance readings
        calibration_data: List of [distance, azimuth, elevation] from calibration
        current_azimuth_steps: Current stepper position in steps
        current_elevation: Current servo elevation in degrees
        azimuth_start: Minimum azimuth in degrees
        azimuth_end: Maximum azimuth in degrees
        elevation_start: Minimum elevation in degrees
        elevation_end: Maximum elevation in degrees
        anomaly_threshold_factor: Factor for anomaly detection (0.7 = 30% change)
        consecutive_detections_needed: Number of consecutive detections required
        max_distance_change: Maximum distance change to consider valid
        
    Returns:
        tuple: (anomaly_detected, anomaly_positions, final_azimuth_steps, 
                final_elevation, final_azimuth_degrees)
    """
    
    print(f"Starting anomaly sweep scan...")
    print(f"Azimuth range: {azimuth_start}° to {azimuth_end}°")
    print(f"Elevation range: {elevation_start}° to {elevation_end}°")
    
    # Convert calibration data to lookup dictionary for faster access
    calibration_lookup = {}
    for distance, azimuth, elevation in calibration_data:
        # Round to match resolution (0.5° azimuth, 2° elevation)
        az_key = round(azimuth / 0.5) * 0.5
        el_key = round(elevation / 2.0) * 2.0
        key = (az_key, el_key)
        
        if key not in calibration_lookup:
            calibration_lookup[key] = []
        calibration_lookup[key].append(distance)
    
    # Calculate average reference distances for each position
    reference_distances = {}
    for key, distances in calibration_lookup.items():
        reference_distances[key] = sum(distances) / len(distances)
    
    # Initialize tracking variables
    anomaly_positions = []
    consecutive_anomaly_count = 0
    anomaly_detected = False
    
    # Convert current position to degrees
    current_azimuth_degrees = (current_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
    
    # Clamp sweep ranges to valid limits
    min_azimuth = max(0, azimuth_start)
    max_azimuth = min(360, azimuth_end)
    min_elevation = max(SERVO_SWEEP_START, elevation_start)
    max_elevation = min(SERVO_SWEEP_END, elevation_end)
    
    # Generate elevation positions (2° resolution)
    elevation_positions = []
    el = min_elevation
    while el <= max_elevation:
        elevation_positions.append(el)
        el += 2.0
    
    # Start sweep
    final_azimuth_steps = current_azimuth_steps
    final_elevation = current_elevation
    
    try:
        for elevation in elevation_positions:
            if anomaly_detected:
                break
                
            print(f"Scanning at elevation: {elevation}°")
            
            # Move servo to current elevation
            set_servo_angle(pi, elevation)
            time.sleep(0.3)  # Allow servo to settle
            final_elevation = elevation
            
            # Determine sweep direction (alternate for efficiency)
            sweep_forward = (elevation_positions.index(elevation) % 2 == 0)
            
            if sweep_forward:
                # Sweep from min to max azimuth
                target_azimuth = min_azimuth
                azimuth_range = [az for az in range(int(min_azimuth * 2), int(max_azimuth * 2) + 1)]
                azimuth_range = [az / 2.0 for az in azimuth_range]  # Convert back to 0.5° steps
            else:
                # Sweep from max to min azimuth
                target_azimuth = max_azimuth
                azimuth_range = [az for az in range(int(max_azimuth * 2), int(min_azimuth * 2) - 1, -1)]
                azimuth_range = [az / 2.0 for az in azimuth_range]  # Convert back to 0.5° steps
            
            # Move to starting azimuth position for this elevation
            steps_to_target = int((target_azimuth / 360.0) * STEPS_PER_REVOLUTION)
            steps_to_move = steps_to_target - final_azimuth_steps
            
            # Handle wrap-around for shortest path
            if abs(steps_to_move) > STEPS_PER_REVOLUTION // 2:
                if steps_to_move > 0:
                    steps_to_move -= STEPS_PER_REVOLUTION
                else:
                    steps_to_move += STEPS_PER_REVOLUTION
            
            # Set direction and move to starting position
            if steps_to_move > 0:
                GPIO.output(DIR_PIN, GPIO.HIGH)
            elif steps_to_move < 0:
                GPIO.output(DIR_PIN, GPIO.LOW)
            
            for _ in range(abs(steps_to_move)):
                stepper_step()
            
            final_azimuth_steps = (final_azimuth_steps + steps_to_move) % STEPS_PER_REVOLUTION
            
            # Now sweep through azimuth range
            for i, azimuth in enumerate(azimuth_range):
                if anomaly_detected:
                    break
                
                # Calculate steps for this azimuth position
                if i > 0:
                    # Move one step at a time
                    if sweep_forward:
                        GPIO.output(DIR_PIN, GPIO.HIGH)
                        stepper_step()
                        final_azimuth_steps = (final_azimuth_steps + 1) % STEPS_PER_REVOLUTION
                    else:
                        GPIO.output(DIR_PIN, GPIO.LOW)
                        stepper_step()
                        final_azimuth_steps = (final_azimuth_steps - 1) % STEPS_PER_REVOLUTION
                
                # Get current position for comparison
                current_az = round(azimuth / 0.5) * 0.5
                current_el = round(elevation / 2.0) * 2.0
                reference_key = (current_az, current_el)
                
                # Collect LiDAR readings
                readings = []
                start_time = time.time()
                while len(readings) < 3 and (time.time() - start_time) < 0.1:  # 100ms timeout
                    try:
                        distance = lidar_data_queue.get_nowait()
                        if distance < SENSOR_MAX:
                            readings.append(distance)
                    except queue.Empty:
                        time.sleep(0.01)
                
                if not readings:
                    continue  # No valid readings, skip this position
                
                # Calculate average current reading
                current_distance = sum(readings) / len(readings)
                
                # Check for anomaly
                anomaly_at_position = False

                if reference_key in reference_distances:
                    # Compare to calibration reference
                    reference_distance = reference_distances[reference_key]
                    
                    # Detect anomaly if current reading is significantly different
                    if current_distance < (reference_distance * anomaly_threshold_factor):
                        anomaly_at_position = True
                        print(f"  Anomaly detected at Az:{azimuth:.1f}°, El:{elevation:.1f}° - "
                            f"Current: {current_distance:.1f}cm, Reference: {reference_distance:.1f}cm")
                else:
                    anomaly_at_position = False # No reference data - treat close objects as anomalies
                
                # Track consecutive anomalies
                if anomaly_at_position:
                    consecutive_anomaly_count += 1
                    anomaly_positions.append([azimuth, elevation])
                    
                    if consecutive_anomaly_count >= consecutive_detections_needed:
                        print(f"Anomaly confirmed! {consecutive_anomaly_count} consecutive detections.")
                        anomaly_detected = True
                        break
                else:
                    consecutive_anomaly_count = 0  # Reset counter
        
        # Calculate final azimuth in degrees
        final_azimuth_degrees = (final_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
        
        print(f"Sweep scan complete.")
        print(f"Final position: Az={final_azimuth_degrees:.1f}° ({final_azimuth_steps} steps), El={final_elevation:.1f}°")
        if anomaly_detected:
            print(f"Anomaly detected at {len(anomaly_positions)} positions")
        else:
            print("No anomalies detected during sweep")
        
        # Return individual values as expected by main.py
        return anomaly_detected, anomaly_positions, final_azimuth_steps, final_elevation, final_azimuth_degrees
        
    except Exception as e:
        print(f"Error during sweep scan: {e}")
        # Return individual values on error
        final_azimuth_degrees = (final_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
        return False, [], final_azimuth_steps, final_elevation, final_azimuth_degrees