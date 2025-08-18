# background_scan.py
# Contains all logic behind background scanning (scanning the environment)

# --- Standard Library Imports ---
import RPi.GPIO as GPIO
import time
import queue
import csv
from datetime import datetime
import numpy as np

# --- Constants ---
# Pin Configuration (BCM numbering)
DIR_PIN    = 26
STEP_PIN   = 13
ENABLE_PIN = 23
RESET_PIN  = 19
M0_PIN     = 5
M1_PIN     = 6
M2_PIN     = 0

SERVO_PIN  = 2

# Motor & Movement Settings
STEPS_PER_REVOLUTION = 6400
STEPPER_PULSE_DELAY = 0.0001
MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500

# Stepper and Servo Sweep Settings
STEPPER_SWEEP_DEGREES = 360
STEPS_FOR_SWEEP = int((STEPPER_SWEEP_DEGREES / 360.0) * STEPS_PER_REVOLUTION)
SERVO_SWEEP_START = 0
SERVO_SWEEP_END = 20

# Scanning Settings
DEFAULT_CALIBRATION_DISTANCE = 1200  # cm - adjust based on your environment
SENSOR_MAX = 1200
ANOMALY_MAX_RADIUS = 5.0  # max degree range in which to average points
SWEEP_RANGE = 10
ANOMALY_FACTOR = 0.6
INITIAL_SWEEP_DETECTIONS_COUNT = 5

# Movement offsets
AZIMUTH_AMOUNT = -5
TILT_AMOUNT = 0

# --- Motor Control Functions ---
def set_servo_angle(pi, angle):
    """Calculates the required pulse width and sets the servo to a specific angle."""
    pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

def stepper_step():
    """Single step of the stepper motor."""
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(STEPPER_PULSE_DELAY)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(STEPPER_PULSE_DELAY)

def reset_stepper_pos(stepper_steps_taken):
    """Reset stepper motor position based on steps tracked from beginning."""
    state = GPIO.input(DIR_PIN)
    if state:
        GPIO.output(DIR_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    
    # Add delay after direction change
    time.sleep(0.001)
    
    if stepper_steps_taken > 0:
        for _ in range(stepper_steps_taken):
            stepper_step()
    
    # Reset to known direction state
    GPIO.output(DIR_PIN, GPIO.HIGH)
    time.sleep(0.001)  # Allow direction to settle
    stepper_steps_taken = 0
    print("Stepper pos reset and dir pin set to HIGH")

def move_to_polar_position(pi, target_azimuth, target_elevation, current_azimuth_steps=0):
    """Move to a specific polar coordinate position."""
    print(f"Moving to polar coordinates: Az={target_azimuth:.1f}°, El={target_elevation:.1f}°")
    
    # --- ELEVATION CONTROL (SERVO) ---
    # Check if elevation is within servo limits and clamp if necessary
    if target_elevation < 0 or target_elevation > 180:
        print(f"Warning: Target elevation {target_elevation:.1f}° is outside servo range "
              f"({0}°-{180}°)")
        target_elevation = max(0, min(180, target_elevation))
        print(f"Clamped elevation to: {target_elevation:.1f}°")
    
    # Move servo to target elevation
    set_servo_angle(pi, target_elevation)
    print(f"Servo moved to elevation: {target_elevation:.1f}°")
    
    # --- AZIMUTH CONTROL (STEPPER) ---
    # Normalize target azimuth to 0-360 range
    target_azimuth = target_azimuth % 360.0
    
    # Convert current steps to current azimuth for debugging
    current_azimuth = (current_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
    print(f"Current position: Az={current_azimuth:.1f}° (steps: {current_azimuth_steps})")
    
    # Convert target azimuth to steps
    target_steps = int((target_azimuth / 360.0) * STEPS_PER_REVOLUTION)
    
    # Calculate steps needed to move from current position
    steps_to_move = target_steps - current_azimuth_steps
    
    # Handle wrap-around to find shortest path
    if abs(steps_to_move) > STEPS_PER_REVOLUTION // 2:
        if steps_to_move > 0:
            steps_to_move -= STEPS_PER_REVOLUTION
        else:
            steps_to_move += STEPS_PER_REVOLUTION
    
    print(f"Steps calculation: target_steps={target_steps}, current_steps={current_azimuth_steps}, steps_to_move={steps_to_move}")
    
    # Move stepper motor if movement is needed
    if steps_to_move != 0:
        # Set direction
        if steps_to_move > 0:
            GPIO.output(DIR_PIN, GPIO.HIGH)  # Clockwise
            direction = "CW"
            print(f"Setting direction: CLOCKWISE (HIGH)")
        else:
            GPIO.output(DIR_PIN, GPIO.LOW)   # Counter-clockwise
            direction = "CCW"
            print(f"Setting direction: COUNTER-CLOCKWISE (LOW)")
        
        # CRITICAL: Add delay after direction change to ensure it takes effect
        time.sleep(0.002)  # Increased delay for DRV8825 setup time
        
        print(f"Moving stepper {abs(steps_to_move)} steps {direction} "
              f"(from {current_azimuth_steps} to target {target_steps})")
        
        # Execute steps
        for step in range(abs(steps_to_move)):
            stepper_step()
            # Optional: Add progress feedback for large movements
            if abs(steps_to_move) > 1000 and step % 500 == 0:
                progress = (step / abs(steps_to_move)) * 100
                print(f"  Progress: {progress:.1f}%")
        
        # Update position
        new_azimuth_steps = current_azimuth_steps + steps_to_move
        
        # Keep steps in valid range (0 to STEPS_PER_REVOLUTION-1)
        new_azimuth_steps = new_azimuth_steps % STEPS_PER_REVOLUTION
        
        # Calculate actual azimuth achieved
        actual_azimuth = (new_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
        
        print(f"Stepper moved to azimuth: {actual_azimuth:.1f}° (steps: {new_azimuth_steps})")
        
    else:
        new_azimuth_steps = current_azimuth_steps
        actual_azimuth = (current_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
        print("No stepper movement needed - already at target azimuth")
    
    print(f"Movement complete: Az={actual_azimuth:.1f}°, El={target_elevation:.1f}°")
    
    return actual_azimuth, target_elevation, new_azimuth_steps

# --- Anomaly Detection Functions ---
def get_interpolated_reference_distance(azimuth, elevation, calibration_data):
    """Get interpolated reference distance for a given position."""
    # Simple nearest neighbor interpolation for now
    # This could be improved with bilinear interpolation
    min_distance = float('inf')
    reference_distance = DEFAULT_CALIBRATION_DISTANCE
    
    for data_point in calibration_data:
        cal_distance, cal_azimuth, cal_elevation = data_point
        distance = ((azimuth - cal_azimuth) ** 2 + (elevation - cal_elevation) ** 2) ** 0.5
        
        if distance < min_distance:
            min_distance = distance
            reference_distance = cal_distance
    
    return reference_distance

# --- Scanning Functions ---
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
                # Store as [distance, azimuth, elevation, timestamp] quadruplet
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
        stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, detections_required, "forward"
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
    
    return current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, state_change

# --- Calibration Functions ---
def calibrate_environment(pi, lidar_data_queue):
    """
    Performs systematic calibration by sweeping stepper motor through 180 degrees,
    then incrementing servo by 2 degrees and repeating. Collects distance data
    with corresponding azimuth and elevation positions, averaging readings for
    each 0.5-degree azimuth increment.
    
    Returns:
        list: Array of [average_distance, azimuth, elevation] measurements at 0.5-degree increments
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
    
    # Calculate azimuth increments - changed from 2 to 0.5 degrees
    azimuth_increments = [round(x * 0.5, 1) for x in range(0, int((STEPPER_SWEEP_DEGREES/0.5)+1))]
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
            target_steps = STEPS_FOR_SWEEP
            GPIO.output(DIR_PIN, GPIO.HIGH)
            
            while stepper_steps_taken < target_steps:
                stepper_steps_taken += 1
                
                # Step the stepper motor
                stepper_step()
                
                # Update physical position
                current_physical_azimuth = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                
                # Find the nearest 0.5-degree increment
                nearest_azimuth = round(current_physical_azimuth / 0.5) * 0.5
                if nearest_azimuth > 360:
                    nearest_azimuth = 360.0

                # Collect LiDAR data if available
                try:
                    distance = lidar_data_queue.get_nowait()
                    if distance < SENSOR_MAX:
                        if nearest_azimuth in azimuth_readings:
                            azimuth_readings[nearest_azimuth].append(distance)
                except queue.Empty:
                    pass
            stepper_steps_taken = 0
                    
        else:
            # Reverse sweep: move from current position to 0°
            target_steps = STEPS_FOR_SWEEP
            GPIO.output(DIR_PIN, GPIO.LOW)
            
            while stepper_steps_taken < target_steps:
                stepper_steps_taken += 1
                
                # Step the stepper motor
                stepper_step()
                
                # Update physical position
                current_physical_azimuth = 360 - (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                
                # Find the nearest 0.5-degree increment
                nearest_azimuth = round(current_physical_azimuth / 0.5) * 0.5
                if nearest_azimuth < 0:
                    nearest_azimuth = 0.0
                
                # Collect LiDAR data if available
                try:
                    distance = lidar_data_queue.get_nowait()
                    if distance < SENSOR_MAX:
                        if nearest_azimuth in azimuth_readings:
                            azimuth_readings[nearest_azimuth].append(distance)
                except queue.Empty:
                    pass
            stepper_steps_taken = 0
        
        # Process collected readings for this elevation
        for azimuth in azimuth_increments:
            readings = azimuth_readings[azimuth]
            if readings:  # Only store if we have readings for this position
                # Calculate average distance, filtering out obvious outliers
                if len(readings) > 3:
                    mean_dist = sum(readings) / len(readings)
                    variance = sum((x - mean_dist) ** 2 for x in readings) / len(readings)
                    stdev_dist = variance ** 0.5
                    filtered_readings = [r for r in readings if abs(r - mean_dist) <= 2 * stdev_dist]
                    if filtered_readings:
                        average_distance = sum(filtered_readings) / len(filtered_readings)
                    else:
                        average_distance = mean_dist
                else:
                    average_distance = sum(readings) / len(readings)
                
                # Store measurement with averaged distance and exact position
                calibration_data.append([average_distance, azimuth, elevation])
                print(f"  Azimuth {azimuth}°: {len(readings)} readings, avg = {average_distance:.1f}cm")
    
    # Return motors to starting position [0,0]
    print("Returning motors to starting position [0,0]...")
    
    # Return servo to 0 degrees
    set_servo_angle(pi, 0)
    
    # Return stepper to 0 degrees
    print(stepper_steps_taken)
    reset_stepper_pos(stepper_steps_taken)
    
    print(f"Calibration complete! Collected {len(calibration_data)} averaged measurements.")
    print("Motors returned to starting position [0,0]")
    
    return calibration_data

def save_calibration_data(calibration_data):
    """
    Save calibration data to CSV file with timestamp.
    
    Args:
        calibration_data: List of [distance, azimuth, elevation] measurements
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Save as CSV file
    csv_filename = f"lidar_calibration_{timestamp}.csv"
    try:
        with open(csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Distance_cm', 'Azimuth_deg', 'Elevation_deg'])  # Header
            for row in calibration_data:
                writer.writerow([f"{row[0]:.1f}", f"{row[1]:.1f}", row[2]])
        print(f"✓ CSV data saved to: {csv_filename}")
    except Exception as e:
        print(f"✗ Error saving CSV: {e}")

# --- Main Background Scanning Function ---
def perform_background_scanning(pi, lidar_data_queue, calibration_data=None):
    """
    Main function to perform background scanning of the environment.
    
    Args:
        pi: pigpio instance for servo control
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Optional calibration data for anomaly detection
        
    Returns:
        tuple: (anomaly_detected, anomaly_locations, anomaly_averaged_coords)
    """
    print("=== BACKGROUND SCANNING STARTING ===")
    
    # Initialize variables
    current_azimuth = 0
    current_elevation = 0
    stepper_steps = 0
    anomaly_locations = []
    anomaly_averaged_coords = []
    anomaly_count = 0
    
    # If no calibration data provided, perform calibration first
    if calibration_data is None:
        print("No calibration data provided. Performing calibration...")
        calibration_data = calibrate_environment(pi, lidar_data_queue)
        save_calibration_data(calibration_data)
    
    # Perform scanning sequence
    print("Starting scanning sequence...")
    current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, state_change = perform_scanning_sequence(
        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation,
        stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, INITIAL_SWEEP_DETECTIONS_COUNT
    )
    
    anomaly_detected = state_change
    
    print("=== BACKGROUND SCANNING COMPLETE ===")
    print(f"Anomalies detected: {anomaly_count}")
    print(f"Anomaly locations: {len(anomaly_averaged_coords)}")
    
    return anomaly_detected, anomaly_locations, anomaly_averaged_coords

if __name__ == '__main__':
    print("Background scanning module loaded.")
    print("Use perform_background_scanning() function to start scanning.")
