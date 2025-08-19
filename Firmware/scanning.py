# scanning.py
import RPi.GPIO as GPIO
import time
import queue
import numpy as np
from constants import *
from move_motors import stepper_step, move_to_polar_position, set_servo_angle
from anomaly_check import get_interpolated_reference_distance
from constants import *



def perform_sweep(pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                 stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, 
                 detections_required, direction="forward"):
    
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

        return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, True
    
    # Perform reverse sweep
    current_azimuth, stepper_steps, anomaly_count, state_change = perform_sweep(
        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation,
        stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, detections_required, "reverse" 
    )
    
    # Final check for state change
    if anomaly_count >= detections_required:
        print(anomaly_count, detections_required)

        state_change = True
    
    return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count , state_change

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
        # Adjust for desired sweep speed
        
        # Get LiDAR reading and process
        try:
            distance = lidar_data_queue.get_nowait()
            
            # Get reference distance for this position
            reference = get_interpolated_reference_distance(current_azimuth, current_elevation, calibration_data)
            
            difference = distance - (reference * ANOMALY_FACTOR)
            
            # Check for anomaly
            if distance < reference * ANOMALY_FACTOR:
                # Store as [distance, azimuth, elevation, timestamp] quadruplet
                anomaly_locations.append([distance, current_azimuth, current_elevation, time.time()])
                
            # Check if we have enough anomalies to declare detection
            if len(anomaly_locations) >= 3:
                # Calculate average of the anomaly group
                avg_anomaly = [sum(col) / len(col) for col in zip(*anomaly_locations)]
                avg_anomaly = [round(val, 2) for val in avg_anomaly]
                anomaly_averaged_coords.append([tuple(avg_anomaly)])
                
                anomaly_locations.clear()  # Clear the list for next group    
                anomaly_count += 1

                 # Clear LiDAR queue before starting
                while not lidar_data_queue.empty():
                    try:
                        lidar_data_queue.get_nowait()
                    except queue.Empty:
                        break
                time.sleep(0.01)
                # Optional: Move to next search area if more detections needed
                if anomaly_count < detections_required and detections_required > 1:
                    start_azimuth += AZIMUTH_AMOUNT
                    end_azimuth += AZIMUTH_AMOUNT
                    start_elevation += TILT_AMOUNT
                    end_elevation += TILT_AMOUNT

                    current_azimuth, current_elevation, stepper_steps = move_to_polar_position(
                        pi, (start_azimuth + end_azimuth)/2, (start_elevation+end_elevation)/2, stepper_steps)
                    
            # Check if we should change state during the sweep
            if anomaly_count >= detections_required:
                
                return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, True, start_azimuth, end_azimuth, start_elevation, end_elevation
                
        except queue.Empty:
            continue
        except Exception as e:
            
            continue
    
    
    return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, False, start_azimuth, end_azimuth, start_elevation, end_elevation


def perform_continuous_servo_scan(pi, lidar_data_queue, calibration_data, current_azimuth, 
                                current_elevation, stepper_steps, tilt_max, tilt_min, 
                                sweep_count=1, servo_speed=20.0):
    """
    Perform servo sweeps in complete cycles. Each sweep cycle goes:
    current_elevation → tilt_max → tilt_min → current_elevation
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees (fixed during scan)
        current_elevation: Current elevation position in degrees (starting/ending position)
        stepper_steps: Current stepper motor step count (unchanged)
        tilt_max: Maximum tilt angle in degrees
        tilt_min: Minimum tilt angle in degrees
        sweep_count: Number of complete sweep cycles to perform (default: 1)
        servo_speed: Servo movement speed in degrees per second (default: 3.0)
        
    Returns:
        tuple: (current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_detected)
            - current_azimuth: Unchanged azimuth position
            - current_elevation: Final elevation position (same as starting)
            - stepper_steps: Unchanged stepper step count
            - anomaly_averaged_coords: List of detected anomaly coordinates
            - anomaly_detected: Boolean indicating if any anomalies were found
    """
    
    print(f"Starting {sweep_count} servo sweep cycle(s)")
    print(f"Sweep pattern: {current_elevation}° → {tilt_max}° → {tilt_min}° → {current_elevation}°")
    print(f"Fixed azimuth: {current_azimuth:.1f}°, Servo speed: {servo_speed}°/s")
    
    # Validate input angles
    tilt_min = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, tilt_min))
    tilt_max = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, tilt_max))
    original_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, current_elevation))
    
    if tilt_min >= tilt_max:
        print("Warning: tilt_min >= tilt_max, swapping values")
        tilt_min, tilt_max = tilt_max, tilt_min
    
    print(f"Validated angles: Start/End={original_elevation}°, Range=[{tilt_min}°, {tilt_max}°]")
    
    # Clear LiDAR queue before starting
    while not lidar_data_queue.empty():
        try:
            lidar_data_queue.get_nowait()
        except queue.Empty:
            break
    
    # Initialize anomaly tracking
    anomaly_locations = []
    anomaly_averaged_coords = []
    anomaly_detected = False
    
    # Move to starting position
    print(f"Moving to starting position: {original_elevation}°")
    set_servo_angle(pi, original_elevation)
    time.sleep(0.2)  # Allow servo to settle
    
    # Perform sweep cycles
    for cycle in range(sweep_count):
        print(f"\n--- Sweep Cycle {cycle + 1}/{sweep_count} ---")
        
        # Define the 3 segments of each sweep cycle
        sweep_segments = [
            (original_elevation, tilt_max, f"Moving to MAX ({tilt_max}°)"),
            (tilt_max, tilt_min, f"Moving to MIN ({tilt_min}°)"),
            (tilt_min, original_elevation, f"Returning to START ({original_elevation}°)")
        ]
        
        for segment_idx, (start_angle, end_angle, description) in enumerate(sweep_segments):
            print(f"  {description}")
            
            # Calculate smooth motion for this segment
            angle_diff = end_angle - start_angle
            distance = abs(angle_diff)
            movement_time = distance / servo_speed
            
            # Number of steps for smooth motion (50ms updates)
            update_interval = 0.05
            num_steps = max(1, int(movement_time / update_interval))
            
            # Create smooth angle progression
            angle_progression = np.linspace(start_angle, end_angle, num_steps)
            
            # Execute smooth movement for this segment
            for step_idx, target_angle in enumerate(angle_progression):
                # Move servo to target position
                set_servo_angle(pi, target_angle)
                
                # Process LiDAR readings during movement
                segment_start_time = time.time()
                readings_collected = 0
                
                while (time.time() - segment_start_time) < update_interval:
                    try:
                        distance = lidar_data_queue.get_nowait()
                        readings_collected += 1
                        
                        # Get reference distance for current position
                        reference = get_interpolated_reference_distance(
                            current_azimuth, target_angle, calibration_data
                        )
                        
                        # Check for anomaly
                        if distance < reference * ANOMALY_FACTOR:
                            detection_time = time.time()
                            anomaly_data = [distance, current_azimuth, target_angle, detection_time]
                            anomaly_locations.append(anomaly_data)
                            
                            print(f"    Anomaly detected: {distance:.1f}cm at "
                                  f"Az={current_azimuth:.1f}°, El={target_angle:.1f}°, "
                                  f"Expected: {reference:.1f}cm")
                            
                            anomaly_detected = True
                            
                    except queue.Empty:
                        time.sleep(0.002)  # Small delay to prevent busy waiting
                        continue
                
                # Progress indication for longer segments
                if num_steps > 10 and step_idx % (num_steps // 4) == 0:
                    progress = (step_idx / num_steps) * 100
                    print(f"    Segment progress: {progress:.0f}% (readings: {readings_collected})")
            
            print(f"  Completed: {description}")
        
        # Process anomalies after each complete cycle
        if len(anomaly_locations) >= 3:
            # Calculate average of the anomaly group
            avg_anomaly = [sum(col) / len(col) for col in zip(*anomaly_locations)]
            avg_anomaly = [round(val, 2) for val in avg_anomaly]
            anomaly_averaged_coords.append(tuple(avg_anomaly))
            
            print(f"  Cycle {cycle + 1} anomaly group averaged: {avg_anomaly}")
            anomaly_locations.clear()  # Clear for next cycle
        
        print(f"Cycle {cycle + 1} complete - returned to starting position")
    
    # Process any remaining anomalies
    if len(anomaly_locations) > 0:
        avg_anomaly = [sum(col) / len(col) for col in zip(*anomaly_locations)]
        avg_anomaly = [round(val, 2) for val in avg_anomaly]
        anomaly_averaged_coords.append(tuple(avg_anomaly))
        print(f"Final anomaly group averaged: {avg_anomaly}")
    
    # Ensure we're at the original position
    final_elevation = original_elevation
    
    print(f"\nSweep sequence complete!")
    print(f"Completed {sweep_count} full cycle(s)")
    print(f"Total anomaly groups detected: {len(anomaly_averaged_coords)}")
    print(f"Final servo position: {final_elevation:.1f}° (returned to start)")
    
    return current_azimuth, final_elevation, stepper_steps, anomaly_averaged_coords, anomaly_detected


def perform_simple_servo_scan(pi, lidar_data_queue, calibration_data, current_azimuth, 
                             current_elevation, stepper_steps, tilt_max, tilt_min, 
                             sweep_count=1, steps_per_segment=15):
    """
    Simplified version with discrete steps instead of time-based smooth motion.
    Each sweep cycle: current_elevation → tilt_max → tilt_min → current_elevation
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees (fixed during scan)
        current_elevation: Current elevation position in degrees (starting/ending position)
        stepper_steps: Current stepper motor step count (unchanged)
        tilt_max: Maximum tilt angle in degrees
        tilt_min: Minimum tilt angle in degrees
        sweep_count: Number of complete sweep cycles to perform (default: 1)
        steps_per_segment: Number of intermediate positions per segment (default: 15)
        
    Returns:
        tuple: Same as perform_continuous_servo_scan
    """
    
    print(f"Starting {sweep_count} simple servo sweep cycle(s)")
    print(f"Pattern: {current_elevation}° → {tilt_max}° → {tilt_min}° → {current_elevation}°")
    
    # Validate angles
    tilt_min = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, tilt_min))
    tilt_max = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, tilt_max))
    original_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, current_elevation))
    
    # Clear LiDAR queue
    while not lidar_data_queue.empty():
        try:
            lidar_data_queue.get_nowait()
        except queue.Empty:
            break
    
    # Initialize tracking
    anomaly_locations = []
    anomaly_averaged_coords = []
    anomaly_detected = False
    
    # Move to starting position
    set_servo_angle(pi, original_elevation)
    time.sleep(0.1)
    
    for cycle in range(sweep_count):
        print(f"Cycle {cycle + 1}/{sweep_count}")
        
        # Three segments per cycle
        segments = [
            np.linspace(original_elevation, tilt_max, steps_per_segment),
            np.linspace(tilt_max, tilt_min, steps_per_segment),
            np.linspace(tilt_min, original_elevation, steps_per_segment)
        ]
        
        for segment_idx, angles in enumerate(segments):
            for angle in angles:
                set_servo_angle(pi, angle)
                time.sleep(0.03)  # Fixed 30ms dwell time
                
                # Process LiDAR reading
                try:
                    distance = lidar_data_queue.get_nowait()
                    reference = get_interpolated_reference_distance(
                        current_azimuth, angle, calibration_data
                    )
                    
                    if distance < reference * ANOMALY_FACTOR:
                        detection_time = time.time()
                        anomaly_data = [distance, current_azimuth, angle, detection_time]
                        anomaly_locations.append(anomaly_data)
                        anomaly_detected = True
                        
                except queue.Empty:
                    continue
        
        # Process anomalies after each cycle
        if len(anomaly_locations) >= 3:
            avg_anomaly = [sum(col) / len(col) for col in zip(*anomaly_locations)]
            avg_anomaly = [round(val, 2) for val in avg_anomaly]
            anomaly_averaged_coords.append(tuple(avg_anomaly))
            anomaly_locations.clear()
    
    # Process final anomalies
    if len(anomaly_locations) > 0:
        avg_anomaly = [sum(col) / len(col) for col in zip(*anomaly_locations)]
        avg_anomaly = [round(val, 2) for val in avg_anomaly]
        anomaly_averaged_coords.append(tuple(avg_anomaly))
    
    print(f"Simple scan complete! {len(anomaly_averaged_coords)} anomaly groups detected")
    
    return current_azimuth, original_elevation, stepper_steps, anomaly_averaged_coords, anomaly_detected