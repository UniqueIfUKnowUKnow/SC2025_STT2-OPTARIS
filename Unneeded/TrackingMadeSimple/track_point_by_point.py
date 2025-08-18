# track_point_by_point.py
# After every found point, searches in an area around it to find the next one and repeats
# Should "predict" the movement

# --- Standard Library Imports ---
import RPi.GPIO as GPIO
import time
import queue
import numpy as np
from datetime import datetime
import math

# Import from our modules
import firmware_StpInz as firmware
import background_scan

# --- Constants ---
# Pin Configuration (BCM numbering)
# DIR_PIN    = 26
# STEP_PIN   = 13
# ENABLE_PIN = 23
# RESET_PIN  = 19
# M0_PIN     = 5
# M1_PIN     = 6
# M2_PIN     = 0
# SERVO_PIN  = 2

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

# Tracking Settings
DEFAULT_CALIBRATION_DISTANCE = 1200  # cm
SENSOR_MAX = 1200
ANOMALY_FACTOR = 0.6

# Prediction Settings
PREDICTION_TIME_STEP = 0.5  # seconds
MAX_PREDICTION_TIME = 5.0   # maximum time to predict ahead
VELOCITY_SMOOTHING_FACTOR = 0.7  # for velocity estimation
POSITION_SMOOTHING_FACTOR = 0.8  # for position smoothing

# Search Settings
INITIAL_SEARCH_RADIUS = 2.0  # degrees
MAX_SEARCH_RADIUS = 10.0     # degrees
SEARCH_RADIUS_MULTIPLIER = 1.5  # multiply radius if target not found
MAX_TRACKING_ATTEMPTS = 5    # maximum attempts before expanding search

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

# --- Position and Velocity Classes ---
class DronePosition:
    """Class to store drone position data."""
    def __init__(self, azimuth, elevation, distance, timestamp):
        self.azimuth = azimuth
        self.elevation = elevation
        self.distance = distance
        self.timestamp = timestamp
    
    def __str__(self):
        return f"Az={self.azimuth:.1f}°, El={self.elevation:.1f}°, Dist={self.distance:.1f}cm, Time={self.timestamp}"

class DroneVelocity:
    """Class to store drone velocity data."""
    def __init__(self, azimuth_velocity, elevation_velocity, distance_velocity):
        self.azimuth_velocity = azimuth_velocity      # degrees/second
        self.elevation_velocity = elevation_velocity  # degrees/second
        self.distance_velocity = distance_velocity    # cm/second
    
    def __str__(self):
        return f"Az_vel={self.azimuth_velocity:.2f}°/s, El_vel={self.elevation_velocity:.2f}°/s, Dist_vel={self.distance_velocity:.1f}cm/s"

# --- Prediction Functions ---
def calculate_velocity(position1, position2):
    """
    Calculate velocity between two positions.
    
    Args:
        position1: First DronePosition
        position2: Second DronePosition
        
    Returns:
        DroneVelocity object
    """
    time_diff = position2.timestamp - position1.timestamp
    if time_diff <= 0:
        return DroneVelocity(0, 0, 0)
    
    # Calculate azimuth velocity (handle wrap-around)
    az_diff = position2.azimuth - position1.azimuth
    if az_diff > 180:
        az_diff -= 360
    elif az_diff < -180:
        az_diff += 360
    azimuth_velocity = az_diff / time_diff
    
    # Calculate elevation velocity
    elevation_velocity = (position2.elevation - position1.elevation) / time_diff
    
    # Calculate distance velocity
    distance_velocity = (position2.distance - position1.distance) / time_diff
    
    return DroneVelocity(azimuth_velocity, elevation_velocity, distance_velocity)

def predict_next_position(current_position, velocity, time_ahead=PREDICTION_TIME_STEP):
    """
    Predict the next position based on current position and velocity.
    
    Args:
        current_position: Current DronePosition
        velocity: Current DroneVelocity
        time_ahead: Time to predict ahead in seconds
        
    Returns:
        tuple: (predicted_azimuth, predicted_elevation, predicted_distance)
    """
    # Predict azimuth (handle wrap-around)
    predicted_azimuth = current_position.azimuth + velocity.azimuth_velocity * time_ahead
    predicted_azimuth = predicted_azimuth % 360.0
    
    # Predict elevation (clamp to servo limits)
    predicted_elevation = current_position.elevation + velocity.elevation_velocity * time_ahead
    predicted_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, predicted_elevation))
    
    # Predict distance (ensure positive)
    predicted_distance = current_position.distance + velocity.distance_velocity * time_ahead
    predicted_distance = max(10, min(SENSOR_MAX, predicted_distance))
    
    return predicted_azimuth, predicted_elevation, predicted_distance

def smooth_velocity(old_velocity, new_velocity, smoothing_factor=VELOCITY_SMOOTHING_FACTOR):
    """
    Smooth velocity using exponential moving average.
    
    Args:
        old_velocity: Previous DroneVelocity
        new_velocity: New DroneVelocity
        smoothing_factor: Smoothing factor (0-1)
        
    Returns:
        Smoothed DroneVelocity
    """
    if old_velocity is None:
        return new_velocity
    
    smoothed_az_vel = (smoothing_factor * old_velocity.azimuth_velocity + 
                       (1 - smoothing_factor) * new_velocity.azimuth_velocity)
    smoothed_el_vel = (smoothing_factor * old_velocity.elevation_velocity + 
                       (1 - smoothing_factor) * new_velocity.elevation_velocity)
    smoothed_dist_vel = (smoothing_factor * old_velocity.distance_velocity + 
                         (1 - smoothing_factor) * new_velocity.distance_velocity)
    
    return DroneVelocity(smoothed_az_vel, smoothed_el_vel, smoothed_dist_vel)

# --- Search Functions ---
def search_at_position(pi, lidar_data_queue, calibration_data, azimuth, elevation, 
                      max_samples=30, detection_threshold_factor=ANOMALY_FACTOR):
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
    max_wait_time = 2.0  # Maximum time to wait for samples
    
    while sample_count < max_samples and (time.time() - start_time) < max_wait_time:
        try:
            distance = lidar_data_queue.get(timeout=0.1)
            
            if distance > 0 and distance < SENSOR_MAX:
                sample_count += 1
                
                # Check if this reading indicates target presence
                if distance < detection_threshold:
                    anomaly_readings.append(distance)
                
                # Early detection: if we have enough anomaly readings, we can conclude
                if len(anomaly_readings) >= 5:
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
        if anomaly_ratio >= 0.4:  # Need at least 40% anomalous readings for tracking
            avg_distance = sum(anomaly_readings) / len(anomaly_readings)
            return True, avg_distance
    
    return False, None

def search_around_position(pi, lidar_data_queue, calibration_data, center_azimuth, center_elevation, 
                          current_azimuth_steps, search_radius, search_points=8):
    """
    Search around a position in a circular pattern.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Calibration data for anomaly detection
        center_azimuth: Center azimuth for search
        center_elevation: Center elevation for search
        current_azimuth_steps: Current stepper steps
        search_radius: Search radius in degrees
        search_points: Number of points to search around the circle
        
    Returns:
        tuple: (found, position, distance, new_azimuth_steps) or (False, None, None, current_azimuth_steps)
    """
    print(f"Searching around position: Az={center_azimuth:.1f}°, El={center_elevation:.1f}°, Radius={search_radius:.1f}°")
    
    # Search at center first
    current_azimuth, current_elevation, current_azimuth_steps = move_to_polar_position(
        pi, center_azimuth, center_elevation, current_azimuth_steps
    )
    
    found, distance = search_at_position(pi, lidar_data_queue, calibration_data, 
                                       current_azimuth, current_elevation)
    if found:
        print(f"Target found at center: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°, Distance={distance:.1f}cm")
        return True, (current_azimuth, current_elevation), distance, current_azimuth_steps
    
    # Search around the circle
    for i in range(search_points):
        angle = (i * 360) / search_points
        search_azimuth = center_azimuth + search_radius * math.cos(math.radians(angle))
        search_elevation = center_elevation + search_radius * math.sin(math.radians(angle))
        
        # Clamp elevation to servo limits
        search_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, search_elevation))
        
        # Move to search position
        current_azimuth, current_elevation, current_azimuth_steps = move_to_polar_position(
            pi, search_azimuth, search_elevation, current_azimuth_steps
        )
        
        # Search at this position
        found, distance = search_at_position(pi, lidar_data_queue, calibration_data, 
                                           current_azimuth, current_elevation)
        
        if found:
            print(f"Target found at search point {i+1}: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°, Distance={distance:.1f}cm")
            return True, (current_azimuth, current_elevation), distance, current_azimuth_steps
    
    print("Target not found in search area")
    return False, None, None, current_azimuth_steps

# --- Tracking Functions ---
def track_single_point(pi, lidar_data_queue, calibration_data, current_position, current_velocity, 
                      current_azimuth_steps):
    """
    Track a single point by predicting its next position and searching there.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Calibration data for anomaly detection
        current_position: Current DronePosition
        current_velocity: Current DroneVelocity
        current_azimuth_steps: Current stepper steps
        
    Returns:
        tuple: (found, new_position, new_velocity, new_azimuth_steps) or (False, None, None, current_azimuth_steps)
    """
    print(f"Tracking point: {current_position}")
    print(f"Current velocity: {current_velocity}")
    
    # Predict next position
    predicted_az, predicted_el, predicted_dist = predict_next_position(current_position, current_velocity)
    print(f"Predicted next position: Az={predicted_az:.1f}°, El={predicted_el:.1f}°, Dist={predicted_dist:.1f}cm")
    
    # Move to predicted position
    current_azimuth, current_elevation, current_azimuth_steps = move_to_polar_position(
        pi, predicted_az, predicted_el, current_azimuth_steps
    )
    
    # Search at predicted position
    found, distance = search_at_position(pi, lidar_data_queue, calibration_data, 
                                       current_azimuth, current_elevation)
    
    if found:
        # Target found at predicted position
        new_position = DronePosition(current_azimuth, current_elevation, distance, time.time())
        new_velocity = calculate_velocity(current_position, new_position)
        smoothed_velocity = smooth_velocity(current_velocity, new_velocity)
        
        print(f"Target found at predicted position: {new_position}")
        print(f"New velocity: {smoothed_velocity}")
        
        return True, new_position, smoothed_velocity, current_azimuth_steps
    else:
        # Target not found at predicted position, search around
        search_radius = INITIAL_SEARCH_RADIUS
        attempts = 0
        
        while attempts < MAX_TRACKING_ATTEMPTS and search_radius <= MAX_SEARCH_RADIUS:
            found, position, distance, current_azimuth_steps = search_around_position(
                pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation,
                current_azimuth_steps, search_radius
            )
            
            if found:
                # Target found in search area
                new_position = DronePosition(position[0], position[1], distance, time.time())
                new_velocity = calculate_velocity(current_position, new_position)
                smoothed_velocity = smooth_velocity(current_velocity, new_velocity)
                
                print(f"Target found in search area: {new_position}")
                print(f"New velocity: {smoothed_velocity}")
                
                return True, new_position, smoothed_velocity, current_azimuth_steps
            
            # Expand search radius
            search_radius *= SEARCH_RADIUS_MULTIPLIER
            attempts += 1
            print(f"Expanding search radius to {search_radius:.1f}° (attempt {attempts})")
        
        print("Target lost - could not find in expanded search area")
        return False, None, None, current_azimuth_steps

# --- Main Tracking Function ---
def track_drone_point_by_point(pi, lidar_data_queue, calibration_data, initial_position, 
                              max_tracking_points=50, tracking_interval=1.0):
    """
    Main function to track drone point by point.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Calibration data for anomaly detection
        initial_position: Initial DronePosition to start tracking from
        max_tracking_points: Maximum number of points to track
        tracking_interval: Time interval between tracking attempts
        
    Returns:
        list: List of tracked DronePosition objects
    """
    print("=== DRONE TRACKING STARTING ===")
    print(f"Initial position: {initial_position}")
    print(f"Tracking started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    tracked_positions = [initial_position]
    current_position = initial_position
    current_velocity = None
    current_azimuth_steps = int((initial_position.azimuth / 360.0) * STEPS_PER_REVOLUTION)
    
    tracking_count = 0
    
    try:
        while tracking_count < max_tracking_points:
            tracking_count += 1
            print(f"\n=== TRACKING POINT {tracking_count} ===")
            
            # Track single point
            found, new_position, new_velocity, current_azimuth_steps = track_single_point(
                pi, lidar_data_queue, calibration_data, current_position, current_velocity, 
                current_azimuth_steps
            )
            
            if found:
                # Update tracking data
                tracked_positions.append(new_position)
                current_position = new_position
                current_velocity = new_velocity
                
                print(f"Successfully tracked point {tracking_count}")
                print(f"Total positions tracked: {len(tracked_positions)}")
                
                # Wait before next tracking attempt
                time.sleep(tracking_interval)
            else:
                print(f"Lost track at point {tracking_count}")
                break
        
        print(f"\n=== TRACKING COMPLETED ===")
        print(f"Total positions tracked: {len(tracked_positions)}")
        print(f"Tracking ended at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        return tracked_positions
        
    except Exception as e:
        print(f"Error during tracking: {e}")
        return tracked_positions

if __name__ == '__main__':
    print("Point-by-point tracking module loaded.")
    print("Use track_drone_point_by_point() function to start tracking.")
