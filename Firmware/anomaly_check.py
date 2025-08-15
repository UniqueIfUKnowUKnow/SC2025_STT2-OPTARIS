# calibration_lookup.py
import math
from constants import DEFAULT_CALIBRATION_DISTANCE, ANOMALY_MAX_RADIUS

# Default distance to use when no calibration point is found within range


def find_nearest_calibration_points(current_azimuth, current_elevation, calibration_data):
    """
    Find the 4 nearest calibration points around a given position.
    Returns distances for the 4 quadrants: (+az,+el), (+az,-el), (-az,+el), (-az,-el)
    
    Args:
        current_azimuth: Current azimuth angle in degrees
        current_elevation: Current elevation angle in degrees  
        calibration_data: List of [distance, azimuth, elevation] from calibration
        ANOMALY_MAX_RADIUS: Maximum distance in degrees to search for points (default 3.0)
        
    Returns:
        dict: {'plus_az_plus_el': distance, 'plus_az_minus_el': distance, 
               'minus_az_plus_el': distance, 'minus_az_minus_el': distance}
    """
    
    # Initialize result dictionary with default values
    result = {
        'plus_az_plus_el': DEFAULT_CALIBRATION_DISTANCE,
        'plus_az_minus_el': DEFAULT_CALIBRATION_DISTANCE,
        'minus_az_plus_el': DEFAULT_CALIBRATION_DISTANCE,
        'minus_az_minus_el': DEFAULT_CALIBRATION_DISTANCE
    }
    
    # Lists to store candidates for each quadrant
    quadrant_candidates = {
        'plus_az_plus_el': [],
        'plus_az_minus_el': [],
        'minus_az_plus_el': [],
        'minus_az_minus_el': []
    }
    
    # Search through all calibration points
    for cal_distance, cal_azimuth, cal_elevation in calibration_data:
        
        # Calculate angular differences
        az_diff = cal_azimuth - current_azimuth
        el_diff = cal_elevation - current_elevation
        
        # Handle azimuth wrap-around (e.g., 359° to 1°)
        if az_diff > 180:
            az_diff -= 360
        elif az_diff < -180:
            az_diff += 360
            
        # Calculate total angular distance
        angular_distance = math.sqrt(az_diff**2 + el_diff**2)
        
        # Skip points outside search radius
        if angular_distance > ANOMALY_MAX_RADIUS:
            continue
            
        # Determine which quadrant this point belongs to
        if az_diff >= 0 and el_diff >= 0:
            quadrant = 'plus_az_plus_el'
        elif az_diff >= 0 and el_diff < 0:
            quadrant = 'plus_az_minus_el'
        elif az_diff < 0 and el_diff >= 0:
            quadrant = 'minus_az_plus_el'
        else:  # az_diff < 0 and el_diff < 0
            quadrant = 'minus_az_minus_el'
            
        # Add to candidates with distance info
        quadrant_candidates[quadrant].append((angular_distance, cal_distance))
    
    # Find the nearest point in each quadrant
    for quadrant in quadrant_candidates:
        if quadrant_candidates[quadrant]:
            # Sort by angular distance and take the closest
            quadrant_candidates[quadrant].sort(key=lambda x: x[0])
            nearest_distance = quadrant_candidates[quadrant][0][1]  # Get the calibration distance
            result[quadrant] = nearest_distance
    
    return result

def get_interpolated_reference_distance(current_azimuth, current_elevation, calibration_data):
    """
    Get an interpolated reference distance based on the 4 nearest calibration points.
    This provides a single reference value for anomaly detection.
    
    Args:
        current_azimuth: Current azimuth angle in degrees
        current_elevation: Current elevation angle in degrees
        calibration_data: List of [distance, azimuth, elevation] from calibration  
        ANOMALY_MAX_RADIUS: Maximum distance in degrees to search for points
        
    Returns:
        float: Interpolated reference distance in cm
    """
    
    # Get the 4 nearest points
    nearest_points = find_nearest_calibration_points(current_azimuth, current_elevation, 
                                                   calibration_data)
    
    # Simple average of the 4 quadrant distances
    distances = list(nearest_points.values())
    reference_distance = sum(distances) / len(distances)
    
    return reference_distance

def find_closest_single_point(current_azimuth, current_elevation, calibration_data):
    """
    Find the single closest calibration point to the current position.
    Simpler alternative if you just need the nearest neighbor.
    
    Args:
        current_azimuth: Current azimuth angle in degrees
        current_elevation: Current elevation angle in degrees
        calibration_data: List of [distance, azimuth, elevation] from calibration
        ANOMALY_MAX_RADIUS: Maximum distance in degrees to search for points
        
    Returns:
        float: Distance from closest calibration point, or DEFAULT_CALIBRATION_DISTANCE if none found
    """
    
    closest_distance = float('inf')
    closest_cal_distance = DEFAULT_CALIBRATION_DISTANCE
    
    for cal_distance, cal_azimuth, cal_elevation in calibration_data:
        # Calculate angular differences
        az_diff = cal_azimuth - current_azimuth
        el_diff = cal_elevation - current_elevation
        
        # Handle azimuth wrap-around
        if az_diff > 180:
            az_diff -= 360
        elif az_diff < -180:
            az_diff += 360
            
        # Calculate total angular distance
        angular_distance = math.sqrt(az_diff**2 + el_diff**2)
        
        # Check if this is the closest point within range
        if angular_distance < closest_distance and angular_distance <= ANOMALY_MAX_RADIUS:
            closest_distance = angular_distance
            closest_cal_distance = cal_distance
    
    return closest_cal_distance

# Example usage:
if __name__ == "__main__":
    # Example calibration data: [distance, azimuth, elevation]
    sample_calibration = [
        [150.0, 45.0, 10.0],
        [200.0, 47.0, 12.0], 
        [180.0, 43.0, 8.0],
        [160.0, 45.5, 10.5]
    ]
    
    # Test the functions
    current_az = 45.0
    current_el = 10.0
    
    # Get 4 quadrant points
    quadrant_points = find_nearest_calibration_points(current_az, current_el, sample_calibration)
    print("Quadrant distances:", quadrant_points)
    
    # Get interpolated reference
    ref_distance = get_interpolated_reference_distance(current_az, current_el, sample_calibration)
    print(f"Interpolated reference distance: {ref_distance:.1f} cm")
    
    # Get single closest point
    closest = find_closest_single_point(current_az, current_el, sample_calibration)
    print(f"Closest single point distance: {closest:.1f} cm")