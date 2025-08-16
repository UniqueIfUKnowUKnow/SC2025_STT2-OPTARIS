import math

def xyz_to_polar(x, y, z):
    
    #Convert Cartesian coordinates (x, y, z) to polar coordinates (azimuth, elevation).
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