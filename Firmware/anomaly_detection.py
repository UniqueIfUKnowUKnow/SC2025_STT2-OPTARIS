# anomaly_detection.py
# Simplified anomaly detection for drone tracking

def get_reference_distance(current_azimuth, current_elevation, calibration_data):
    """
    Get a reference distance from calibration data for the current position.
    Uses simple nearest neighbor approach.
    
    Args:
        current_azimuth: Current azimuth angle in degrees
        current_elevation: Current elevation angle in degrees  
        calibration_data: List of [distance, azimuth, elevation] from calibration
        
    Returns:
        float: Reference distance in cm
    """
    if not calibration_data:
        return 1200  # Default distance if no calibration data
    
    # Find the closest calibration point
    min_distance = float('inf')
    reference_distance = 1200
    
    for cal_distance, cal_azimuth, cal_elevation in calibration_data:
        # Calculate angular distance
        az_diff = abs(cal_azimuth - current_azimuth)
        el_diff = abs(cal_elevation - current_elevation)
        
        # Handle azimuth wrap-around
        if az_diff > 180:
            az_diff = 360 - az_diff
            
        # Total angular distance
        angular_distance = (az_diff**2 + el_diff**2)**0.5
        
        if angular_distance < min_distance:
            min_distance = angular_distance
            reference_distance = cal_distance
    
    return reference_distance

def is_anomaly(distance, reference_distance, anomaly_factor=0.6):
    """
    Check if a distance reading indicates an anomaly (potential drone).
    
    Args:
        distance: Current LiDAR distance reading in cm
        reference_distance: Expected distance from calibration in cm
        anomaly_factor: Factor to determine anomaly threshold
        
    Returns:
        bool: True if anomaly detected, False otherwise
    """
    threshold = reference_distance * anomaly_factor
    return distance < threshold
