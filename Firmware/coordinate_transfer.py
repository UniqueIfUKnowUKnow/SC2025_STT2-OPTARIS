import math

def xyz_to_polar(x, y, z):
    """
    Convert Cartesian coordinates (x, y, z) to polar coordinates (distance, azimuth, elevation).
    
    Args:
        x (float): X coordinate (forward/backward relative to sensor)
        y (float): Y coordinate (left/right relative to sensor) 
        z (float): Z coordinate (up/down relative to sensor)
    
    Returns:
        tuple: (azimuth_degrees, elevation_degrees, distance)
            - azimuth: 0-360 degrees (0° = +X axis, 90° = +Y axis)
            - elevation: -90 to +90 degrees (0° = horizontal, +90° = straight up)
            - distance: radial distance from origin
    """
    # Calculate radial distance
    distance = math.sqrt(x**2 + y**2 + z**2)
    
    if distance == 0:
        return 0.0, 0.0, 0.0
    
    # Calculate azimuth (angle in XY plane from X-axis)
    azimuth_rad = math.atan2(y, x)
    azimuth_deg = math.degrees(azimuth_rad)
    
    # Ensure azimuth is in 0-360 range
    if azimuth_deg < 0:
        azimuth_deg += 360
    
    # Calculate elevation (angle from XY plane)
    horizontal_distance = math.sqrt(x**2 + y**2)
    if horizontal_distance == 0:
        # Point is directly above or below
        elevation_deg = 90.0 if z > 0 else -90.0
    else:
        elevation_rad = math.atan2(z, horizontal_distance)
        elevation_deg = math.degrees(elevation_rad)
    
    return azimuth_deg, elevation_deg, distance