# data_processing.py
# Processes and outputs the following data:
# - TLE coordinate data
# - current position
# - the time of detection of the last position(current position)
# - velocity of the target
# - based on the velocity and direction, it should try and predict its next movement
# - if it the drone is not in the predicted movement, the system starts scanning around the last known position in a wider range to try and find the target

# --- Standard Library Imports ---
import json
import csv
import time
import numpy as np
from datetime import datetime, timedelta
import math
import ephem
from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional

# Import from our modules
import track_point_by_point
from track_point_by_point import DronePosition, DroneVelocity

# --- Constants ---
# TLE Data (ISS example - can be updated with actual drone TLE)
DEFAULT_TLE_DATA = {
    "name": "ISS (ZARYA)",
    "line1": "1 25544U 98067A   24225.51782528  .00016717  00000-0  30199-3 0  9992",
    "line2": "2 25544  51.6426  95.0936 0007310  10.0000  60.6990 15.50209251454141"
}

# Coordinate Conversion Settings
EARTH_RADIUS = 6371000  # meters
LIDAR_HEIGHT = 1.5      # meters (height of LiDAR above ground)

# Prediction Settings
PREDICTION_TIME_STEPS = [0.5, 1.0, 2.0, 5.0]  # seconds to predict ahead
VELOCITY_THRESHOLD = 0.1  # minimum velocity to consider movement
POSITION_UNCERTAINTY = 2.0  # degrees - uncertainty in position measurement

# Output Settings
OUTPUT_FORMATS = ["json", "csv", "tle", "summary"]
DEFAULT_OUTPUT_DIR = "drone_data"

# --- Data Classes ---
@dataclass
class TLEData:
    """TLE data structure."""
    name: str
    line1: str
    line2: str
    epoch: Optional[datetime] = None
    
    def __post_init__(self):
        if self.epoch is None:
            # Parse epoch from TLE line1
            try:
                year = int(self.line1[18:20])
                day = float(self.line1[20:32])
                if year < 57:  # 1957-2056
                    year += 2000
                else:
                    year += 1900
                self.epoch = datetime(year, 1, 1) + timedelta(days=day-1)
            except:
                self.epoch = datetime.now()

@dataclass
class ProcessedDroneData:
    """Processed drone data structure."""
    timestamp: datetime
    position: DronePosition
    velocity: Optional[DroneVelocity]
    tle_data: TLEData
    predicted_positions: List[Tuple[float, float, float]]  # (azimuth, elevation, distance) for different time steps
    confidence: float  # confidence in current position (0-1)
    status: str  # "tracking", "searching", "lost"

# --- TLE Processing Functions ---
def parse_tle_data(tle_data_dict):
    """
    Parse TLE data dictionary into TLEData object.
    
    Args:
        tle_data_dict: Dictionary containing TLE data
        
    Returns:
        TLEData object
    """
    return TLEData(
        name=tle_data_dict.get("name", "Unknown"),
        line1=tle_data_dict.get("line1", ""),
        line2=tle_data_dict.get("line2", "")
    )

def calculate_tle_position(tle_data, timestamp):
    """
    Calculate TLE-based position for a given timestamp.
    
    Args:
        tle_data: TLEData object
        timestamp: datetime object
        
    Returns:
        tuple: (azimuth, elevation, distance) in degrees and meters
    """
    try:
        # Create ephem satellite object
        sat = ephem.readtle(tle_data.name, tle_data.line1, tle_data.line2)
        
        # Set observer (you would need to set actual observer coordinates)
        observer = ephem.Observer()
        observer.lat = '0.0'  # Replace with actual latitude
        observer.lon = '0.0'  # Replace with actual longitude
        observer.elevation = LIDAR_HEIGHT
        observer.date = timestamp
        
        # Calculate satellite position
        sat.compute(observer)
        
        # Convert to degrees and distance
        azimuth = math.degrees(sat.az)
        elevation = math.degrees(sat.alt)
        distance = sat.range / 1000.0  # Convert to km
        
        return azimuth, elevation, distance
        
    except Exception as e:
        print(f"Error calculating TLE position: {e}")
        return None, None, None

# --- Coordinate Conversion Functions ---
def polar_to_cartesian(azimuth, elevation, distance):
    """
    Convert polar coordinates to Cartesian coordinates.
    
    Args:
        azimuth: Azimuth angle in degrees
        elevation: Elevation angle in degrees
        distance: Distance in cm
        
    Returns:
        tuple: (x, y, z) in meters
    """
    # Convert to radians
    az_rad = math.radians(azimuth)
    el_rad = math.radians(elevation)
    
    # Convert distance to meters
    distance_m = distance / 100.0
    
    # Calculate Cartesian coordinates
    x = distance_m * math.cos(el_rad) * math.sin(az_rad)
    y = distance_m * math.cos(el_rad) * math.cos(az_rad)
    z = distance_m * math.sin(el_rad) + LIDAR_HEIGHT
    
    return x, y, z

def cartesian_to_polar(x, y, z):
    """
    Convert Cartesian coordinates to polar coordinates.
    
    Args:
        x, y, z: Cartesian coordinates in meters
        
    Returns:
        tuple: (azimuth, elevation, distance) in degrees and cm
    """
    # Calculate distance
    distance = math.sqrt(x**2 + y**2 + (z - LIDAR_HEIGHT)**2)
    
    # Calculate azimuth
    azimuth = math.degrees(math.atan2(x, y))
    if azimuth < 0:
        azimuth += 360
    
    # Calculate elevation
    elevation = math.degrees(math.asin((z - LIDAR_HEIGHT) / distance))
    
    # Convert distance to cm
    distance_cm = distance * 100.0
    
    return azimuth, elevation, distance_cm

def calculate_confidence(position, velocity, recent_positions):
    """
    Calculate confidence in current position based on consistency.
    
    Args:
        position: Current DronePosition
        velocity: Current DroneVelocity
        recent_positions: List of recent DronePosition objects
        
    Returns:
        float: Confidence value (0-1)
    """
    if not recent_positions or len(recent_positions) < 2:
        return 0.5  # Default confidence
    
    # Calculate position consistency
    position_errors = []
    for prev_pos in recent_positions[-3:]:  # Last 3 positions
        az_error = abs(position.azimuth - prev_pos.azimuth)
        if az_error > 180:
            az_error = 360 - az_error
        el_error = abs(position.elevation - prev_pos.elevation)
        dist_error = abs(position.distance - prev_pos.distance) / 100.0  # Normalize
        
        total_error = math.sqrt(az_error**2 + el_error**2 + dist_error**2)
        position_errors.append(total_error)
    
    avg_error = np.mean(position_errors)
    position_confidence = max(0, 1 - avg_error / 10.0)  # Normalize to 0-1
    
    # Calculate velocity consistency
    if velocity:
        vel_magnitude = math.sqrt(velocity.azimuth_velocity**2 + 
                                velocity.elevation_velocity**2 + 
                                (velocity.distance_velocity/100.0)**2)
        velocity_confidence = min(1.0, vel_magnitude / 10.0)  # Normalize
    else:
        velocity_confidence = 0.5
    
    # Combined confidence
    confidence = (position_confidence * 0.7 + velocity_confidence * 0.3)
    return max(0, min(1, confidence))

# --- Prediction Functions ---
def predict_future_positions(position, velocity, time_steps=PREDICTION_TIME_STEPS):
    """
    Predict future positions based on current position and velocity.
    
    Args:
        position: Current DronePosition
        velocity: Current DroneVelocity
        time_steps: List of time steps to predict ahead
        
    Returns:
        List of predicted positions as (azimuth, elevation, distance) tuples
    """
    if not velocity:
        return [(position.azimuth, position.elevation, position.distance)] * len(time_steps)
    
    predicted_positions = []
    
    for time_step in time_steps:
        # Predict azimuth (handle wrap-around)
        predicted_az = position.azimuth + velocity.azimuth_velocity * time_step
        predicted_az = predicted_az % 360.0
        
        # Predict elevation (clamp to reasonable limits)
        predicted_el = position.elevation + velocity.elevation_velocity * time_step
        predicted_el = max(-90, min(90, predicted_el))
        
        # Predict distance (ensure positive)
        predicted_dist = position.distance + velocity.distance_velocity * time_step
        predicted_dist = max(10, min(1200, predicted_dist))
        
        predicted_positions.append((predicted_az, predicted_el, predicted_dist))
    
    return predicted_positions

def check_prediction_accuracy(actual_position, predicted_positions, time_steps):
    """
    Check accuracy of predictions against actual position.
    
    Args:
        actual_position: Actual DronePosition
        predicted_positions: List of predicted positions
        time_steps: List of time steps used for prediction
        
    Returns:
        List of accuracy scores (0-1)
    """
    accuracies = []
    
    for i, (pred_az, pred_el, pred_dist) in enumerate(predicted_positions):
        # Calculate position error
        az_error = abs(actual_position.azimuth - pred_az)
        if az_error > 180:
            az_error = 360 - az_error
        
        el_error = abs(actual_position.elevation - pred_el)
        dist_error = abs(actual_position.distance - pred_dist) / 100.0  # Normalize
        
        # Calculate total error
        total_error = math.sqrt(az_error**2 + el_error**2 + dist_error**2)
        
        # Convert to accuracy (0-1)
        accuracy = max(0, 1 - total_error / 10.0)
        accuracies.append(accuracy)
    
    return accuracies

# --- Data Processing Functions ---
def process_drone_data(position, velocity, tle_data, recent_positions=None):
    """
    Process drone data and create comprehensive output.
    
    Args:
        position: Current DronePosition
        velocity: Current DroneVelocity
        tle_data: TLEData object
        recent_positions: List of recent positions for confidence calculation
        
    Returns:
        ProcessedDroneData object
    """
    # Calculate confidence
    confidence = calculate_confidence(position, velocity, recent_positions or [])
    
    # Predict future positions
    predicted_positions = predict_future_positions(position, velocity)
    
    # Determine status
    if velocity and abs(velocity.azimuth_velocity) + abs(velocity.elevation_velocity) > VELOCITY_THRESHOLD:
        status = "tracking"
    elif confidence > 0.7:
        status = "searching"
    else:
        status = "lost"
    
    return ProcessedDroneData(
        timestamp=datetime.fromtimestamp(position.timestamp),
        position=position,
        velocity=velocity,
        tle_data=tle_data,
        predicted_positions=predicted_positions,
        confidence=confidence,
        status=status
    )

# --- Output Functions ---
def save_data_to_json(processed_data, filename=None):
    """
    Save processed data to JSON file.
    
    Args:
        processed_data: ProcessedDroneData object
        filename: Optional filename
        
    Returns:
        str: Filename of saved file
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"drone_data_{timestamp}.json"
    
    # Convert to dictionary
    data_dict = {
        "timestamp": processed_data.timestamp.isoformat(),
        "position": {
            "azimuth": processed_data.position.azimuth,
            "elevation": processed_data.position.elevation,
            "distance": processed_data.position.distance,
            "timestamp": processed_data.position.timestamp
        },
        "velocity": None if processed_data.velocity is None else {
            "azimuth_velocity": processed_data.velocity.azimuth_velocity,
            "elevation_velocity": processed_data.velocity.elevation_velocity,
            "distance_velocity": processed_data.velocity.distance_velocity
        },
        "tle_data": {
            "name": processed_data.tle_data.name,
            "line1": processed_data.tle_data.line1,
            "line2": processed_data.tle_data.line2,
            "epoch": processed_data.tle_data.epoch.isoformat() if processed_data.tle_data.epoch else None
        },
        "predicted_positions": processed_data.predicted_positions,
        "confidence": processed_data.confidence,
        "status": processed_data.status
    }
    
    try:
        with open(filename, 'w') as f:
            json.dump(data_dict, f, indent=2)
        print(f"✓ Data saved to JSON: {filename}")
        return filename
    except Exception as e:
        print(f"✗ Error saving JSON: {e}")
        return None

def save_data_to_csv(processed_data, filename=None):
    """
    Save processed data to CSV file.
    
    Args:
        processed_data: ProcessedDroneData object
        filename: Optional filename
        
    Returns:
        str: Filename of saved file
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"drone_data_{timestamp}.csv"
    
    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            writer.writerow([
                'Timestamp', 'Azimuth_deg', 'Elevation_deg', 'Distance_cm',
                'Azimuth_Velocity_deg_s', 'Elevation_Velocity_deg_s', 'Distance_Velocity_cm_s',
                'Confidence', 'Status', 'TLE_Name'
            ])
            
            # Write data
            velocity_data = [0, 0, 0] if processed_data.velocity is None else [
                processed_data.velocity.azimuth_velocity,
                processed_data.velocity.elevation_velocity,
                processed_data.velocity.distance_velocity
            ]
            
            writer.writerow([
                processed_data.timestamp.isoformat(),
                f"{processed_data.position.azimuth:.2f}",
                f"{processed_data.position.elevation:.2f}",
                f"{processed_data.position.distance:.1f}",
                f"{velocity_data[0]:.3f}",
                f"{velocity_data[1]:.3f}",
                f"{velocity_data[2]:.1f}",
                f"{processed_data.confidence:.3f}",
                processed_data.status,
                processed_data.tle_data.name
            ])
        
        print(f"✓ Data saved to CSV: {filename}")
        return filename
    except Exception as e:
        print(f"✗ Error saving CSV: {e}")
        return None

def save_tle_data(processed_data, filename=None):
    """
    Save TLE data to file.
    
    Args:
        processed_data: ProcessedDroneData object
        filename: Optional filename
        
    Returns:
        str: Filename of saved file
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"tle_data_{timestamp}.txt"
    
    try:
        with open(filename, 'w') as f:
            f.write(f"{processed_data.tle_data.name}\n")
            f.write(f"{processed_data.tle_data.line1}\n")
            f.write(f"{processed_data.tle_data.line2}\n")
            if processed_data.tle_data.epoch:
                f.write(f"Epoch: {processed_data.tle_data.epoch.isoformat()}\n")
        
        print(f"✓ TLE data saved to: {filename}")
        return filename
    except Exception as e:
        print(f"✗ Error saving TLE: {e}")
        return None

def print_summary(processed_data):
    """
    Print summary of processed data.
    
    Args:
        processed_data: ProcessedDroneData object
    """
    print("\n" + "="*60)
    print("DRONE DATA SUMMARY")
    print("="*60)
    print(f"Timestamp: {processed_data.timestamp}")
    print(f"Status: {processed_data.status.upper()}")
    print(f"Confidence: {processed_data.confidence:.1%}")
    
    print(f"\nCurrent Position:")
    print(f"  Azimuth: {processed_data.position.azimuth:.1f}°")
    print(f"  Elevation: {processed_data.position.elevation:.1f}°")
    print(f"  Distance: {processed_data.position.distance:.1f}cm")
    
    if processed_data.velocity:
        print(f"\nCurrent Velocity:")
        print(f"  Azimuth: {processed_data.velocity.azimuth_velocity:.2f}°/s")
        print(f"  Elevation: {processed_data.velocity.elevation_velocity:.2f}°/s")
        print(f"  Distance: {processed_data.velocity.distance_velocity:.1f}cm/s")
    
    print(f"\nTLE Data:")
    print(f"  Name: {processed_data.tle_data.name}")
    print(f"  Epoch: {processed_data.tle_data.epoch}")
    
    print(f"\nPredicted Positions:")
    time_steps = PREDICTION_TIME_STEPS
    for i, (az, el, dist) in enumerate(processed_data.predicted_positions):
        print(f"  {time_steps[i]}s: Az={az:.1f}°, El={el:.1f}°, Dist={dist:.1f}cm")
    
    print("="*60)

# --- Main Processing Function ---
def process_and_output_drone_data(position, velocity, tle_data_dict=None, recent_positions=None, 
                                output_formats=None):
    """
    Main function to process and output drone data.
    
    Args:
        position: Current DronePosition
        velocity: Current DroneVelocity
        tle_data_dict: TLE data dictionary (uses default if None)
        recent_positions: List of recent positions for confidence calculation
        output_formats: List of output formats (uses default if None)
        
    Returns:
        ProcessedDroneData object
    """
    print("=== PROCESSING DRONE DATA ===")
    
    # Parse TLE data
    if tle_data_dict is None:
        tle_data_dict = DEFAULT_TLE_DATA
    tle_data = parse_tle_data(tle_data_dict)
    
    # Process data
    processed_data = process_drone_data(position, velocity, tle_data, recent_positions)
    
    # Set output formats
    if output_formats is None:
        output_formats = OUTPUT_FORMATS
    
    # Output data
    saved_files = []
    
    if "json" in output_formats:
        json_file = save_data_to_json(processed_data)
        if json_file:
            saved_files.append(json_file)
    
    if "csv" in output_formats:
        csv_file = save_data_to_csv(processed_data)
        if csv_file:
            saved_files.append(csv_file)
    
    if "tle" in output_formats:
        tle_file = save_tle_data(processed_data)
        if tle_file:
            saved_files.append(tle_file)
    
    if "summary" in output_formats:
        print_summary(processed_data)
    
    print(f"\n=== PROCESSING COMPLETE ===")
    print(f"Files saved: {len(saved_files)}")
    for file in saved_files:
        print(f"  - {file}")
    
    return processed_data

if __name__ == '__main__':
    print("Data processing module loaded.")
    print("Use process_and_output_drone_data() function to process drone data.")
    print("Available output formats:", OUTPUT_FORMATS)
