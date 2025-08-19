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
                                scan_duration=10.0, servo_speed=2.0):
    """
    Continuously sweep the servo between specified angles while detecting anomalies.
    The servo moves smoothly up and down while LiDAR data is continuously monitored.
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees (fixed during scan)
        current_elevation: Current elevation position in degrees
        stepper_steps: Current stepper motor step count (unchanged)
        tilt_max: Maximum tilt angle in degrees
        tilt_min: Minimum tilt angle in degrees
        scan_duration: Total scan duration in seconds (default: 10.0)
        servo_speed: Servo movement speed in degrees per second (default: 2.0)
        
    Returns:
        tuple: (current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_detected)
            - current_azimuth: Unchanged azimuth position
            - current_elevation: Final elevation position
            - stepper_steps: Unchanged stepper step count
            - anomaly_averaged_coords: List of detected anomaly coordinates
            - anomaly_detected: Boolean indicating if any anomalies were found
    """
    
    print(f"Starting continuous servo scan between {tilt_min}° and {tilt_max}°")
    print(f"Fixed azimuth: {current_azimuth:.1f}°, Scan duration: {scan_duration}s")
    
    # Validate input angles
    tilt_min = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, tilt_min))
    tilt_max = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, tilt_max))
    
    if tilt_min >= tilt_max:
        print("Warning: tilt_min >= tilt_max, swapping values")
        tilt_min, tilt_max = tilt_max, tilt_min
    
    print(f"Validated tilt range: {tilt_min}° to {tilt_max}°")
    
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
    
    # Calculate servo motion parameters
    tilt_range = tilt_max - tilt_min
    update_interval = 0.05  # 50ms updates for smooth motion
    updates_per_second = 1.0 / update_interval
    total_updates = int(scan_duration * updates_per_second)
    
    # Create smooth servo motion pattern
    # Use a sine wave pattern for continuous up-down motion
    start_time = time.time()
    current_servo_angle = current_elevation
    
    # Move to starting position
    set_servo_angle(pi, current_servo_angle)
    time.sleep(0.1)  # Allow servo to settle
    
    print(f"Beginning continuous scan with {total_updates} servo updates...")
    
    for update_count in range(total_updates):
        current_time = time.time()
        elapsed_time = current_time - start_time
        
        # Calculate smooth servo position using sine wave
        # This creates continuous up-down motion
        cycle_frequency = servo_speed / tilt_range  # Cycles per second
        angle_progress = np.sin(2 * np.pi * cycle_frequency * elapsed_time)
        
        # Map sine wave (-1 to 1) to servo range (tilt_min to tilt_max)
        target_servo_angle = tilt_min + (angle_progress + 1) * 0.5 * tilt_range
        
        # Ensure angle is within bounds
        target_servo_angle = max(tilt_min, min(tilt_max, target_servo_angle))
        
        # Move servo smoothly to target position
        set_servo_angle(pi, target_servo_angle)
        current_servo_angle = target_servo_angle
        
        # Process LiDAR readings during motion
        lidar_readings_this_cycle = []
        
        # Collect multiple readings during this update interval
        cycle_start = time.time()
        while (time.time() - cycle_start) < update_interval:
            try:
                distance = lidar_data_queue.get_nowait()
                lidar_readings_this_cycle.append(distance)
            except queue.Empty:
                time.sleep(0.001)  # Small delay to prevent busy waiting
                continue
        
        # Process collected LiDAR readings
        for distance in lidar_readings_this_cycle:
            # Get reference distance for current position
            reference = get_interpolated_reference_distance(
                current_azimuth, current_servo_angle, calibration_data
            )
            
            # Check for anomaly
            if distance < reference * ANOMALY_FACTOR:
                detection_time = time.time()
                anomaly_data = [distance, current_azimuth, current_servo_angle, detection_time]
                anomaly_locations.append(anomaly_data)
                
                print(f"Anomaly detected: {distance:.1f}cm at "
                      f"Az={current_azimuth:.1f}°, El={current_servo_angle:.1f}°, "
                      f"Expected: {reference:.1f}cm")
                
                anomaly_detected = True
        
        # Group anomalies if we have enough detections
        if len(anomaly_locations) >= 3:
            # Calculate average of the anomaly group
            avg_anomaly = [sum(col) / len(col) for col in zip(*anomaly_locations)]
            avg_anomaly = [round(val, 2) for val in avg_anomaly]
            anomaly_averaged_coords.append(tuple(avg_anomaly))
            
            print(f"Anomaly group averaged: {avg_anomaly}")
            anomaly_locations.clear()  # Clear for next group
        
        # Optional: Print progress every 2 seconds
        if update_count % (int(2.0 / update_interval)) == 0:
            progress = (elapsed_time / scan_duration) * 100
            print(f"Scan progress: {progress:.1f}% - Current tilt: {current_servo_angle:.1f}°")
    
    # Process any remaining anomalies
    if len(anomaly_locations) > 0:
        avg_anomaly = [sum(col) / len(col) for col in zip(*anomaly_locations)]
        avg_anomaly = [round(val, 2) for val in avg_anomaly]
        anomaly_averaged_coords.append(tuple(avg_anomaly))
        print(f"Final anomaly group averaged: {avg_anomaly}")
    
    # Final servo position
    final_elevation = current_servo_angle
    
    print(f"Continuous servo scan complete!")
    print(f"Total anomaly groups detected: {len(anomaly_averaged_coords)}")
    print(f"Final servo position: {final_elevation:.1f}°")
    
    return current_azimuth, final_elevation, stepper_steps, anomaly_averaged_coords, anomaly_detected


def perform_bidirectional_servo_scan(pi, lidar_data_queue, calibration_data, current_azimuth, 
                                    current_elevation, stepper_steps, tilt_max, tilt_min, 
                                    num_cycles=3, dwell_time=0.02):
    """
    Alternative scanning pattern that moves servo up and down in discrete cycles.
    Provides more predictable motion pattern compared to continuous sine wave.
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees (fixed during scan)
        current_elevation: Current elevation position in degrees
        stepper_steps: Current stepper motor step count (unchanged)
        tilt_max: Maximum tilt angle in degrees
        tilt_min: Minimum tilt angle in degrees
        num_cycles: Number of up-down cycles to perform (default: 3)
        dwell_time: Time to pause at each servo position in seconds (default: 0.02)
        
    Returns:
        tuple: Same as perform_continuous_servo_scan
    """
    
    print(f"Starting bidirectional servo scan: {num_cycles} cycles between {tilt_min}° and {tilt_max}°")
    
    # Validate input angles
    tilt_min = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, tilt_min))
    tilt_max = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, tilt_max))
    
    if tilt_min >= tilt_max:
        tilt_min, tilt_max = tilt_max, tilt_min
    
    # Clear LiDAR queue
    while not lidar_data_queue.empty():
        try:
            lidar_data_queue.get_nowait()
        except queue.Empty:
            break
    
    # Initialize anomaly tracking
    anomaly_locations = []
    anomaly_averaged_coords = []
    anomaly_detected = False
    
    # Create servo positions for smooth motion
    steps_per_sweep = 20  # Number of intermediate positions
    
    for cycle in range(num_cycles):
        print(f"Cycle {cycle + 1}/{num_cycles}")
        
        # Upward sweep
        tilt_positions_up = np.linspace(tilt_min, tilt_max, steps_per_sweep)
        for tilt_pos in tilt_positions_up:
            set_servo_angle(pi, tilt_pos)
            time.sleep(dwell_time)
            
            # Collect LiDAR readings
            try:
                distance = lidar_data_queue.get_nowait()
                reference = get_interpolated_reference_distance(
                    current_azimuth, tilt_pos, calibration_data
                )
                
                if distance < reference * ANOMALY_FACTOR:
                    detection_time = time.time()
                    anomaly_data = [distance, current_azimuth, tilt_pos, detection_time]
                    anomaly_locations.append(anomaly_data)
                    anomaly_detected = True
                    
            except queue.Empty:
                continue
        
        # Downward sweep
        tilt_positions_down = np.linspace(tilt_max, tilt_min, steps_per_sweep)
        for tilt_pos in tilt_positions_down:
            set_servo_angle(pi, tilt_pos)
            time.sleep(dwell_time)
            
            # Collect LiDAR readings
            try:
                distance = lidar_data_queue.get_nowait()
                reference = get_interpolated_reference_distance(
                    current_azimuth, tilt_pos, calibration_data
                )
                
                if distance < reference * ANOMALY_FACTOR:
                    detection_time = time.time()
                    anomaly_data = [distance, current_azimuth, tilt_pos, detection_time]
                    anomaly_locations.append(anomaly_data)
                    anomaly_detected = True
                    
            except queue.Empty:
                continue
        
        # Process anomalies after each cycle
        if len(anomaly_locations) >= 3:
            avg_anomaly = [sum(col) / len(col) for col in zip(*anomaly_locations)]
            avg_anomaly = [round(val, 2) for val in avg_anomaly]
            anomaly_averaged_coords.append(tuple(avg_anomaly))
            
            print(f"Cycle {cycle + 1} anomaly group: {avg_anomaly}")
            anomaly_locations.clear()
    
    # Process final anomalies
    if len(anomaly_locations) > 0:
        avg_anomaly = [sum(col) / len(col) for col in zip(*anomaly_locations)]
        avg_anomaly = [round(val, 2) for val in avg_anomaly]
        anomaly_averaged_coords.append(tuple(avg_anomaly))
    
    final_elevation = tilt_positions_down[-1]  # Last position from downward sweep
    
    print(f"Bidirectional scan complete! Detected {len(anomaly_averaged_coords)} anomaly groups")
    
    return current_azimuth, final_elevation, stepper_steps, anomaly_averaged_coords, anomaly_detected