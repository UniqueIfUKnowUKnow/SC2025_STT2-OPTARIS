# anomaly_check.py - Simplified radial averaging approach
import math
from constants import DEFAULT_CALIBRATION_DISTANCE, ANOMALY_MAX_RADIUS

def get_reference_distance_radial_average(current_azimuth, current_elevation, calibration_data, max_radius=None):
    """
    Get a reference distance by averaging all calibration points within a specified angular radius.
    
    Args:
        current_azimuth: Current azimuth angle in degrees
        current_elevation: Current elevation angle in degrees  
        calibration_data: List of [distance, azimuth, elevation] from calibration
        max_radius: Maximum angular distance in degrees to include points (default: ANOMALY_MAX_RADIUS)
        
    Returns:
        tuple: (reference_distance, num_points_used)
            - reference_distance: Weighted average distance in cm
            - num_points_used: Number of calibration points included in average
    """
    if max_radius is None:
        max_radius = ANOMALY_MAX_RADIUS
    
    weighted_distances = []
    weights = []
    
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
        
        # Include points within the specified radius
        if angular_distance <= max_radius:
            # Use inverse distance weighting - closer points have more influence
            # Add small epsilon to avoid division by zero for exact matches
            weight = 1.0 / (angular_distance + 0.1)
            
            weighted_distances.append(cal_distance * weight)
            weights.append(weight)
    
    # Calculate weighted average if we have points, otherwise use default
    if weights:
        reference_distance = sum(weighted_distances) / sum(weights)
        num_points_used = len(weights)
    else:
        reference_distance = DEFAULT_CALIBRATION_DISTANCE
        num_points_used = 0
    
    return reference_distance, num_points_used

def get_reference_distance_simple_average(current_azimuth, current_elevation, calibration_data, max_radius=None):
    """
    Get a reference distance using simple averaging (no weighting) of all points within radius.
    
    Args:
        current_azimuth: Current azimuth angle in degrees
        current_elevation: Current elevation angle in degrees  
        calibration_data: List of [distance, azimuth, elevation] from calibration
        max_radius: Maximum angular distance in degrees to include points (default: ANOMALY_MAX_RADIUS)
        
    Returns:
        tuple: (reference_distance, num_points_used)
    """
    if max_radius is None:
        max_radius = ANOMALY_MAX_RADIUS
    
    distances_in_range = []
    
    # Search through all calibration points
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
        
        # Include points within the specified radius
        if angular_distance <= max_radius:
            distances_in_range.append(cal_distance)
    
    # Calculate simple average if we have points, otherwise use default
    if distances_in_range:
        reference_distance = sum(distances_in_range) / len(distances_in_range)
        num_points_used = len(distances_in_range)
    else:
        reference_distance = DEFAULT_CALIBRATION_DISTANCE
        num_points_used = 0
    
    return reference_distance, num_points_used

def find_nearest_calibration_point(current_azimuth, current_elevation, calibration_data):
    """
    Find the single nearest calibration point and return its distance.
    Useful for debugging or as a fallback method.
    
    Args:
        current_azimuth: Current azimuth angle in degrees
        current_elevation: Current elevation angle in degrees
        calibration_data: List of [distance, azimuth, elevation] from calibration
        
    Returns:
        tuple: (distance, angular_distance, azimuth, elevation)
            - distance: Distance from nearest calibration point
            - angular_distance: Angular separation to nearest point
            - azimuth, elevation: Position of nearest calibration point
    """
    
    closest_distance = float('inf')
    closest_cal_distance = DEFAULT_CALIBRATION_DISTANCE
    closest_position = (0, 0)
    
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
        
        # Check if this is the closest point
        if angular_distance < closest_distance:
            closest_distance = angular_distance
            closest_cal_distance = cal_distance
            closest_position = (cal_azimuth, cal_elevation)
    
    return closest_cal_distance, closest_distance, closest_position[0], closest_position[1]

def get_adaptive_reference_distance(current_azimuth, current_elevation, calibration_data):
    """
    Adaptive reference distance calculation that adjusts the search radius based on point density.
    Starts with a small radius and expands until enough points are found.
    
    Args:
        current_azimuth: Current azimuth angle in degrees
        current_elevation: Current elevation angle in degrees
        calibration_data: List of [distance, azimuth, elevation] from calibration
        
    Returns:
        tuple: (reference_distance, radius_used, num_points_used)
    """
    min_points_needed = 3  # Minimum points for a good average
    max_points_needed = 10  # Don't need more than this for averaging
    
    # Try progressively larger radii
    radii_to_try = [2.0, 5.0, 10.0, 15.0, 20.0, 30.0]
    
    for radius in radii_to_try:
        ref_distance, num_points = get_reference_distance_radial_average(
            current_azimuth, current_elevation, calibration_data, radius
        )
        
        if num_points >= min_points_needed:
            return ref_distance, radius, num_points
    
    # If we still don't have enough points, use the largest radius attempted
    ref_distance, num_points = get_reference_distance_radial_average(
        current_azimuth, current_elevation, calibration_data, radii_to_try[-1]
    )
    
    return ref_distance, radii_to_try[-1], num_points

# Convenience function that maintains backward compatibility with your existing code
def get_interpolated_reference_distance(current_azimuth, current_elevation, calibration_data):
    """
    Main function for getting reference distance - maintains compatibility with existing code.
    Uses weighted averaging of all points within ANOMALY_MAX_RADIUS.
    
    Args:
        current_azimuth: Current azimuth angle in degrees
        current_elevation: Current elevation angle in degrees
        calibration_data: List of [distance, azimuth, elevation] from calibration
        
    Returns:
        float: Reference distance in cm
    """
    reference_distance, _ = get_reference_distance_radial_average(
        current_azimuth, current_elevation, calibration_data
    )
    return reference_distance

# Example usage and testing
if __name__ == "__main__":
    # Example calibration data: [distance, azimuth, elevation]
    sample_calibration = [
        [150.0, 45.0, 10.0],
        [200.0, 47.0, 12.0], 
        [180.0, 43.0, 8.0],
        [160.0, 45.5, 10.5],
        [170.0, 44.0, 9.0],
        [190.0, 46.0, 11.0],
        [155.0, 45.2, 10.2]
    ]
    
    # Test position
    current_az = 45.0
    current_el = 10.0
    
    print(f"Testing at position: Az={current_az}°, El={current_el}°")
    print("=" * 50)
    
    # Test weighted averaging
    ref_dist_weighted, num_points_weighted = get_reference_distance_radial_average(
        current_az, current_el, sample_calibration, max_radius=5.0
    )
    print(f"Weighted average (5° radius): {ref_dist_weighted:.1f} cm using {num_points_weighted} points")
    
    # Test simple averaging
    ref_dist_simple, num_points_simple = get_reference_distance_simple_average(
        current_az, current_el, sample_calibration, max_radius=5.0
    )
    print(f"Simple average (5° radius): {ref_dist_simple:.1f} cm using {num_points_simple} points")
    
    # Test adaptive radius
    ref_dist_adaptive, radius_used, num_points_adaptive = get_adaptive_reference_distance(
        current_az, current_el, sample_calibration
    )
    print(f"Adaptive radius: {ref_dist_adaptive:.1f} cm using {num_points_adaptive} points (radius: {radius_used}°)")
    
    # Test nearest neighbor
    nearest_dist, angular_dist, nearest_az, nearest_el = find_nearest_calibration_point(
        current_az, current_el, sample_calibration
    )
    print(f"Nearest neighbor: {nearest_dist:.1f} cm at ({nearest_az}°, {nearest_el}°), distance: {angular_dist:.2f}°")
    
    # Test backward compatibility
    ref_dist_compat = get_interpolated_reference_distance(current_az, current_el, sample_calibration)
    print(f"Backward compatible function: {ref_dist_compat:.1f} cm")