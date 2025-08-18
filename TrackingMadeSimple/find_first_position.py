# find_first_position.py
# Searches for the first position of the drone using systematic scanning patterns

# --- Standard Library Imports ---
import RPi.GPIO as GPIO
import time
import queue
import numpy as np
from datetime import datetime

# Import from our modules
import firmware_StpInz as firmware
import background_scan

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
DEFAULT_CALIBRATION_DISTANCE = 1200  # cm
SENSOR_MAX = 1200
ANOMALY_FACTOR = 0.6
SWEEP_RANGE = 10

# Search Patterns
SEARCH_PATTERNS = {
    "spiral": "Spiral pattern from center outward",
    "grid": "Grid pattern covering the entire area",
    "concentric": "Concentric circles from center",
    "zigzag": "Zigzag pattern covering the area"
}

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

def move_to_polar_position(pi, target_azimuth, target_elevation, current_azimuth_steps=0):
    """Move to a specific polar coordinate position."""
    print(f"Moving to polar coordinates: Az={target_azimuth:.1f}°, El={target_elevation:.1f}°")
    
    # --- ELEVATION CONTROL (SERVO) ---
    if target_elevation < 0 or target_elevation > 180:
        print(f"Warning: Target elevation {target_elevation:.1f}° is outside servo range")
        target_elevation = max(0, min(180, target_elevation))
    
    set_servo_angle(pi, target_elevation)
    
    # --- AZIMUTH CONTROL (STEPPER) ---
    target_azimuth = target_azimuth % 360.0
    current_azimuth = (current_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
    
    target_steps = int((target_azimuth / 360.0) * STEPS_PER_REVOLUTION)
    steps_to_move = target_steps - current_azimuth_steps
    
    # Handle wrap-around to find shortest path
    if abs(steps_to_move) > STEPS_PER_REVOLUTION // 2:
        if steps_to_move > 0:
            steps_to_move -= STEPS_PER_REVOLUTION
        else:
            steps_to_move += STEPS_PER_REVOLUTION
    
    # Move stepper motor if movement is needed
    if steps_to_move != 0:
        if steps_to_move > 0:
            GPIO.output(DIR_PIN, GPIO.HIGH)  # Clockwise
        else:
            GPIO.output(DIR_PIN, GPIO.LOW)   # Counter-clockwise
        
        time.sleep(0.002)  # Delay for direction change
        
        for step in range(abs(steps_to_move)):
            stepper_step()
        
        new_azimuth_steps = current_azimuth_steps + steps_to_move
        new_azimuth_steps = new_azimuth_steps % STEPS_PER_REVOLUTION
    else:
        new_azimuth_steps = current_azimuth_steps
    
    actual_azimuth = (new_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
    return actual_azimuth, target_elevation, new_azimuth_steps

# --- Search Pattern Functions ---
def spiral_search(pi, lidar_data_queue, calibration_data, center_azimuth=180, center_elevation=10, 
                 max_radius=90, step_size=5):
    """
    Perform spiral search pattern from center outward.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Calibration data for anomaly detection
        center_azimuth: Center azimuth for spiral
        center_elevation: Center elevation for spiral
        max_radius: Maximum search radius in degrees
        step_size: Step size for spiral
        
    Returns:
        tuple: (found, position, distance) or (False, None, None)
    """
    print(f"Starting spiral search from center: Az={center_azimuth}°, El={center_elevation}°")
    
    current_azimuth_steps = int((center_azimuth / 360.0) * STEPS_PER_REVOLUTION)
    current_azimuth = center_azimuth
    current_elevation = center_elevation
    
    # Move to center position
    current_azimuth, current_elevation, current_azimuth_steps = move_to_polar_position(
        pi, center_azimuth, center_elevation, current_azimuth_steps
    )
    
    radius = 0
    angle = 0
    angle_step = 15  # degrees per spiral turn
    
    while radius <= max_radius:
        # Calculate spiral position
        spiral_azimuth = center_azimuth + radius * np.cos(np.radians(angle))
        spiral_elevation = center_elevation + radius * np.sin(np.radians(angle))
        
        # Clamp elevation to servo limits
        spiral_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, spiral_elevation))
        
        # Move to spiral position
        current_azimuth, current_elevation, current_azimuth_steps = move_to_polar_position(
            pi, spiral_azimuth, spiral_elevation, current_azimuth_steps
        )
        
        # Search at this position
        found, distance = search_at_position(pi, lidar_data_queue, calibration_data, 
                                           current_azimuth, current_elevation)
        
        if found:
            print(f"DRONE FOUND at spiral position: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°, Distance={distance:.1f}cm")
            return True, (current_azimuth, current_elevation), distance
        
        # Update spiral parameters
        angle += angle_step
        if angle >= 360:
            angle = 0
            radius += step_size
    
    print("Spiral search completed - no drone found")
    return False, None, None

def grid_search(pi, lidar_data_queue, calibration_data, azimuth_range=360, elevation_range=20, 
               azimuth_step=10, elevation_step=5):
    """
    Perform grid search pattern covering the entire area.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Calibration data for anomaly detection
        azimuth_range: Total azimuth range to search
        elevation_range: Total elevation range to search
        azimuth_step: Step size for azimuth
        elevation_step: Step size for elevation
        
    Returns:
        tuple: (found, position, distance) or (False, None, None)
    """
    print(f"Starting grid search: Az={azimuth_range}°, El={elevation_range}°")
    
    current_azimuth_steps = 0
    
    # Start from bottom-left corner
    start_azimuth = 0
    start_elevation = SERVO_SWEEP_START
    
    for elevation in np.arange(start_elevation, start_elevation + elevation_range, elevation_step):
        print(f"Searching elevation: {elevation}°")
        
        for azimuth in np.arange(start_azimuth, start_azimuth + azimuth_range, azimuth_step):
            # Move to grid position
            current_azimuth, current_elevation, current_azimuth_steps = move_to_polar_position(
                pi, azimuth, elevation, current_azimuth_steps
            )
            
            # Search at this position
            found, distance = search_at_position(pi, lidar_data_queue, calibration_data, 
                                               current_azimuth, current_elevation)
            
            if found:
                print(f"DRONE FOUND at grid position: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°, Distance={distance:.1f}cm")
                return True, (current_azimuth, current_elevation), distance
    
    print("Grid search completed - no drone found")
    return False, None, None

def concentric_search(pi, lidar_data_queue, calibration_data, center_azimuth=180, center_elevation=10, 
                     max_radius=90, radius_step=10):
    """
    Perform concentric circle search pattern from center outward.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Calibration data for anomaly detection
        center_azimuth: Center azimuth for circles
        center_elevation: Center elevation for circles
        max_radius: Maximum search radius in degrees
        radius_step: Step size for radius
        
    Returns:
        tuple: (found, position, distance) or (False, None, None)
    """
    print(f"Starting concentric search from center: Az={center_azimuth}°, El={center_elevation}°")
    
    current_azimuth_steps = int((center_azimuth / 360.0) * STEPS_PER_REVOLUTION)
    
    # Move to center position first
    current_azimuth, current_elevation, current_azimuth_steps = move_to_polar_position(
        pi, center_azimuth, center_elevation, current_azimuth_steps
    )
    
    # Search at center
    found, distance = search_at_position(pi, lidar_data_queue, calibration_data, 
                                       current_azimuth, current_elevation)
    if found:
        print(f"DRONE FOUND at center: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°, Distance={distance:.1f}cm")
        return True, (current_azimuth, current_elevation), distance
    
    # Search in concentric circles
    for radius in np.arange(radius_step, max_radius + radius_step, radius_step):
        print(f"Searching circle with radius: {radius}°")
        
        # Search at 8 points around the circle
        for angle in np.arange(0, 360, 45):
            circle_azimuth = center_azimuth + radius * np.cos(np.radians(angle))
            circle_elevation = center_elevation + radius * np.sin(np.radians(angle))
            
            # Clamp elevation to servo limits
            circle_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, circle_elevation))
            
            # Move to circle position
            current_azimuth, current_elevation, current_azimuth_steps = move_to_polar_position(
                pi, circle_azimuth, circle_elevation, current_azimuth_steps
            )
            
            # Search at this position
            found, distance = search_at_position(pi, lidar_data_queue, calibration_data, 
                                               current_azimuth, current_elevation)
            
            if found:
                print(f"DRONE FOUND at circle position: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°, Distance={distance:.1f}cm")
                return True, (current_azimuth, current_elevation), distance
    
    print("Concentric search completed - no drone found")
    return False, None, None

def zigzag_search(pi, lidar_data_queue, calibration_data, azimuth_range=360, elevation_range=20, 
                 azimuth_step=15, elevation_step=5):
    """
    Perform zigzag search pattern covering the area.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Calibration data for anomaly detection
        azimuth_range: Total azimuth range to search
        elevation_range: Total elevation range to search
        azimuth_step: Step size for azimuth
        elevation_step: Step size for elevation
        
    Returns:
        tuple: (found, position, distance) or (False, None, None)
    """
    print(f"Starting zigzag search: Az={azimuth_range}°, El={elevation_range}°")
    
    current_azimuth_steps = 0
    
    for elevation in np.arange(SERVO_SWEEP_START, SERVO_SWEEP_START + elevation_range, elevation_step):
        print(f"Searching elevation: {elevation}°")
        
        # Determine direction for this elevation (zigzag)
        if int(elevation / elevation_step) % 2 == 0:
            # Left to right
            azimuth_sequence = np.arange(0, azimuth_range, azimuth_step)
        else:
            # Right to left
            azimuth_sequence = np.arange(azimuth_range - azimuth_step, -azimuth_step, -azimuth_step)
        
        for azimuth in azimuth_sequence:
            # Move to zigzag position
            current_azimuth, current_elevation, current_azimuth_steps = move_to_polar_position(
                pi, azimuth, elevation, current_azimuth_steps
            )
            
            # Search at this position
            found, distance = search_at_position(pi, lidar_data_queue, calibration_data, 
                                               current_azimuth, current_elevation)
            
            if found:
                print(f"DRONE FOUND at zigzag position: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°, Distance={distance:.1f}cm")
                return True, (current_azimuth, current_elevation), distance
    
    print("Zigzag search completed - no drone found")
    return False, None, None

# --- Detection Functions ---
def search_at_position(pi, lidar_data_queue, calibration_data, azimuth, elevation, 
                      max_samples=20, detection_threshold_factor=ANOMALY_FACTOR):
    """
    Search for drone at a specific position.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Calibration data for anomaly detection
        azimuth: Azimuth position to search
        elevation: Elevation position to search
        max_samples: Maximum number of LiDAR samples to collect
        detection_threshold_factor: Factor to determine if reading is anomalous
        
    Returns:
        tuple: (found, distance) or (False, None)
    """
    # Clear any old readings from queue
    while not lidar_data_queue.empty():
        try:
            lidar_data_queue.get_nowait()
        except queue.Empty:
            break
    
    # Get reference distance for this position
    reference_distance = background_scan.get_interpolated_reference_distance(azimuth, elevation, calibration_data)
    detection_threshold = reference_distance * detection_threshold_factor
    
    # Collect samples at this position
    anomaly_readings = []
    sample_count = 0
    start_time = time.time()
    max_wait_time = 1.0  # Maximum time to wait for samples
    
    while sample_count < max_samples and (time.time() - start_time) < max_wait_time:
        try:
            distance = lidar_data_queue.get(timeout=0.1)
            
            if distance > 0 and distance < SENSOR_MAX:
                sample_count += 1
                
                # Check if this reading indicates target presence
                if distance < detection_threshold:
                    anomaly_readings.append(distance)
                
                # Early detection: if we have enough anomaly readings, we can conclude
                if len(anomaly_readings) >= 3:
                    avg_distance = sum(anomaly_readings) / len(anomaly_readings)
                    return True, avg_distance
                    
        except queue.Empty:
            time.sleep(0.05)
            continue
        except Exception as e:
            print(f"Error reading LiDAR: {e}")
            continue
    
    # Determine if target is present based on anomaly ratio
    if sample_count > 0:
        anomaly_ratio = len(anomaly_readings) / sample_count
        if anomaly_ratio >= 0.3:  # Need at least 30% anomalous readings
            avg_distance = sum(anomaly_readings) / len(anomaly_readings)
            return True, avg_distance
    
    return False, None

# --- Main Search Function ---
def find_first_drone_position(pi, lidar_data_queue, calibration_data, search_pattern="spiral"):
    """
    Main function to find the first position of the drone.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Calibration data for anomaly detection
        search_pattern: Search pattern to use ("spiral", "grid", "concentric", "zigzag")
        
    Returns:
        tuple: (found, position, distance, search_time) or (False, None, None, search_time)
    """
    print("=== DRONE SEARCH STARTING ===")
    print(f"Search pattern: {search_pattern}")
    print(f"Search started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    start_time = time.time()
    
    try:
        # Perform search based on pattern
        if search_pattern == "spiral":
            found, position, distance = spiral_search(pi, lidar_data_queue, calibration_data)
        elif search_pattern == "grid":
            found, position, distance = grid_search(pi, lidar_data_queue, calibration_data)
        elif search_pattern == "concentric":
            found, position, distance = concentric_search(pi, lidar_data_queue, calibration_data)
        elif search_pattern == "zigzag":
            found, position, distance = zigzag_search(pi, lidar_data_queue, calibration_data)
        else:
            print(f"Unknown search pattern: {search_pattern}. Using spiral search.")
            found, position, distance = spiral_search(pi, lidar_data_queue, calibration_data)
        
        search_time = time.time() - start_time
        
        if found:
            print(f"=== DRONE FOUND ===")
            print(f"Position: Az={position[0]:.1f}°, El={position[1]:.1f}°")
            print(f"Distance: {distance:.1f}cm")
            print(f"Search time: {search_time:.2f} seconds")
            return True, position, distance, search_time
        else:
            print(f"=== DRONE NOT FOUND ===")
            print(f"Search time: {search_time:.2f} seconds")
            return False, None, None, search_time
            
    except Exception as e:
        search_time = time.time() - start_time
        print(f"Error during drone search: {e}")
        return False, None, None, search_time

if __name__ == '__main__':
    print("Drone search module loaded.")
    print("Available search patterns:", list(SEARCH_PATTERNS.keys()))
    print("Use find_first_drone_position() function to start searching.")
