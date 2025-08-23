def generate_mock_tle(cos_base, sin_base, n_hat, phase_filter, first_scan_times, original_tle_data=None):
    """
    Generate a mock TLE from phase tracking data, preserving all original TLE information
    except for the inclination angle which is calculated from tracking data.
    
    Args:
        cos_base: C vector (cosine basis in orbital plane) - numpy array [3]
        sin_base: S vector (sine basis in orbital plane) - numpy array [3]
        n_hat: plane normal vector (angular momentum direction) - numpy array [3]
        phase_filter: [s0, Omega] - current phase and angular velocity [rad, rad/s]
        first_scan_times: array of measurement times (unix timestamps)
        original_tle_data: parsed TLE data from tle_processing.parse_tle() (optional)
        
    Returns:
        tuple: (line1, line2) of TLE with checksums
    """
    import numpy as np
    from datetime import datetime, timezone
    import time
    
    # If no original TLE data provided, import from constants
    if original_tle_data is None:
        from constants import satellite_name, line1, line2
        from tle_processing import parse_tle
        original_tle_data = parse_tle([satellite_name, line1, line2])
    
    # Constants for calculations
    k_hat = np.array([0, 0, 1])  # z-axis unit vector (Earth's rotation axis)
    
    # Calculate NEW inclination from tracking data
    inclination_rad = np.arccos(np.clip(np.dot(n_hat, k_hat), -1.0, 1.0))
    inclination_deg = np.degrees(inclination_rad)
    
    # Use ALL original TLE values except inclination
    satellite_number = original_tle_data['satnum']
    classification = original_tle_data['classification']
    intl_designator = original_tle_data['intl_designator']
    
    # Preserve original epoch
    epoch_year_2digit = original_tle_data['epoch_year'] % 100
    epoch_day = original_tle_data['epoch_day']
    
    # Preserve original orbital dynamics
    mean_motion_dot = original_tle_data['mean_motion_dot']
    mean_motion_ddot = original_tle_data['mean_motion_ddot']
    bstar = original_tle_data['bstar']
    ephemeris_type = original_tle_data['ephemeris_type']
    element_set_number = original_tle_data['element_set_number']
    
    # Preserve original orbital elements (except inclination)
    raan_deg = original_tle_data['raan_deg']
    eccentricity = original_tle_data['eccentricity']
    arg_perigee_deg = original_tle_data['arg_perigee_deg'] 
    mean_anomaly_deg = original_tle_data['mean_anomaly_deg']
    mean_motion_rev_per_day = original_tle_data['mean_motion_rev_per_day']
    revolution_number = original_tle_data['rev_number_at_epoch']
    
    # Helper function to format scientific notation for TLE
    def format_scientific_tle(value):
        """Convert float to TLE's implied scientific notation format"""
        if abs(value) < 1e-10:
            return " 00000-0"
        
        # Convert to scientific notation
        if value >= 0:
            sign = "+"
        else:
            sign = "-"
            value = abs(value)
        
        # Find exponent
        if value == 0:
            return " 00000-0"
        
        exponent = int(np.floor(np.log10(value)))
        mantissa = value / (10 ** exponent)
        
        # Format mantissa (remove decimal point, take first 5 digits)
        mantissa_str = f"{mantissa:.5f}".replace(".", "")[:5]
        
        # Handle negative exponent
        if exponent < 0:
            exp_sign = "-"
            exp_val = abs(exponent)
        else:
            exp_sign = "+"
            exp_val = exponent
        
        return f" {mantissa_str}{exp_sign}{exp_val}"
    
    # Format the scientific notation values
    mean_motion_ddot_str = format_scientific_tle(mean_motion_ddot)
    bstar_str = format_scientific_tle(bstar)
    
    # Helper function to calculate TLE checksum
    def calculate_checksum(line):
        """Calculate TLE checksum for a line"""
        total = 0
        for char in line:
            if char.isdigit():
                total += int(char)
            elif char == "-":
                total += 1
        return total % 10
    
    # Format TLE Line 1 (without checksum first)
    line1_no_checksum = (
        f"1 {satellite_number:5d}{classification} "
        f"{intl_designator:8s} "
        f"{epoch_year_2digit:02d}{epoch_day:012.8f} "
        f"{mean_motion_dot:10.8f} "
        f"{mean_motion_ddot_str} "
        f"{bstar_str} "
        f"{ephemeris_type:1d} "
        f"{element_set_number:4d}"
    )
    
    # Calculate and append checksum for line 1
    checksum1 = calculate_checksum(line1_no_checksum)
    line1 = line1_no_checksum + str(checksum1)
    
    # Format TLE Line 2 (without checksum first)
    # Convert eccentricity to integer format (multiply by 10^7, no decimal point)
    ecc_int = int(round(eccentricity * 10000000))
    
    line2_no_checksum = (
        f"2 {satellite_number:5d} "
        f"{inclination_deg:8.4f} "  # THIS IS THE ONLY CHANGED VALUE
        f"{raan_deg:8.4f} "
        f"{ecc_int:07d} "
        f"{arg_perigee_deg:8.4f} "
        f"{mean_anomaly_deg:8.4f} "
        f"{mean_motion_rev_per_day:11.8f}"
        f"{revolution_number:5d}"
    )
    
    # Calculate and append checksum for line 2
    checksum2 = calculate_checksum(line2_no_checksum)
    line2 = line2_no_checksum + str(checksum2)
    
    print(f"Generated updated TLE (preserving original except inclination):")
    print(f"Original inclination: {original_tle_data['inclination_deg']:.4f}°")
    print(f"Calculated inclination: {inclination_deg:.4f}°")
    print(f"Satellite: {original_tle_data['name'] if 'name' in original_tle_data else 'Unknown'}")
    print(f"NORAD ID: {satellite_number}")
    print(f"Epoch: {original_tle_data['epoch_year']}-{epoch_day:.8f}")
    print(f"Mean Motion: {mean_motion_rev_per_day:.8f} rev/day")
    
    return line1, line2