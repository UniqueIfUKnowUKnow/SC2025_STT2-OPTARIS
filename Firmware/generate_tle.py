def generate_mock_tle(cos_base, sin_base, n_hat, phase_filter, first_scan_times, satellite_name="TRACKED_OBJ"):
    """
    Generate a mock TLE from phase tracking data as outlined in section 3.5.13 of the PDF.
    
    Args:
        cos_base: C vector (cosine basis in orbital plane) - numpy array [3]
        sin_base: S vector (sine basis in orbital plane) - numpy array [3]
        n_hat: plane normal vector (angular momentum direction) - numpy array [3]
        phase_filter: [s0, Omega] - current phase and angular velocity [rad, rad/s]
        first_scan_times: array of measurement times (unix timestamps)
        satellite_name: name for the TLE (max 24 chars)
        
    Returns:
        tuple: (satellite_name, line1, line2) of TLE
    """
    import numpy as np
    from datetime import datetime, timezone
    import time
    
    # Extract phase and angular velocity
    s0 = phase_filter[0]  # current phase in radians
    Omega = phase_filter[1]  # angular velocity in rad/s
    
    # Constants for mock satellite
    Rsim = 6900.0  # Representative LEO radius in km (ISS-like)
    k_hat = np.array([0, 0, 1])  # z-axis unit vector (Earth's rotation axis)
    
    # Calculate orbital elements from plane normal n_hat
    # Inclination: i = arccos(n_hat · k)
    inclination_rad = np.arccos(np.clip(np.dot(n_hat, k_hat), -1.0, 1.0))
    inclination_deg = np.degrees(inclination_rad)
    
    # Line of nodes vector: N = k × n_hat
    N = np.cross(k_hat, n_hat)
    N_mag = np.linalg.norm(N)
    
    if N_mag > 1e-10:  # Not polar orbit
        N = N / N_mag
        # RAAN: Ω_RAAN = atan2(Ny, Nx) 
        raan_rad = np.arctan2(N[1], N[0])
        raan_deg = np.degrees(raan_rad) % 360.0
    else:  # Polar orbit case
        raan_deg = 0.0
    
    # Orbital elements (for circular orbit)
    eccentricity = 0.0  # Circular by construction
    arg_perigee_deg = 0.0  # Argument of perigee = 0
    
    # Mean anomaly at epoch: M0 = wrap[0,2π)(s0)
    mean_anomaly_deg = np.degrees(s0 % (2*np.pi))
    
    # Mean motion: n = (86400/2π)Ω (rev/day)
    mean_motion_rev_per_day = (86400.0 / (2*np.pi)) * abs(Omega)
    
    # Epoch time (use first measurement time or current time)
    if len(first_scan_times) > 0:
        epoch_time = first_scan_times[0]
    else:
        epoch_time = time.time()
    
    epoch_dt = datetime.fromtimestamp(epoch_time, tz=timezone.utc)
    
    # Convert to TLE epoch format (YY + day of year with fraction)
    year = epoch_dt.year
    year_2digit = year % 100
    day_of_year = epoch_dt.timetuple().tm_yday
    hour_fraction = (epoch_dt.hour + epoch_dt.minute/60.0 + 
                    epoch_dt.second/3600.0 + epoch_dt.microsecond/3600000000.0) / 24.0
    epoch_day = day_of_year + hour_fraction
    
    # TLE parameters
    satellite_number = 99999  # Dummy satellite number
    classification = "U"  # Unclassified
    intl_designator = "25001A"  # Dummy international designator  
    mean_motion_dot = 0.0  # First derivative of mean motion
    mean_motion_ddot_exp = " 00000-0"  # Second derivative in implied scientific notation
    bstar_exp = " 00000-0"  # Drag coefficient in implied scientific notation
    ephemeris_type = 0
    element_set_number = 1
    revolution_number = 1
    
    # Truncate satellite name to fit TLE format
    sat_name = satellite_name[:24].ljust(24)
    
    # Format TLE Line 1 (columns as per TLE specification)
    line1_data = (
        f"1 {satellite_number:5d}{classification} "
        f"{intl_designator:8s} "
        f"{year_2digit:02d}{epoch_day:012.8f} "
        f"{mean_motion_dot:10.8f} "
        f"{mean_motion_ddot_exp} "
        f"{bstar_exp} "
        f"{ephemeris_type:1d} "
        f"{element_set_number:4d}"
    )
    
    # Calculate checksum for line 1 (must be exactly 68 chars before checksum)
    line1_68 = line1_data[:68]
    checksum1 = calculate_tle_checksum(line1_68)
    line1 = line1_68 + str(checksum1)
    
    # Format TLE Line 2 (columns as per TLE specification)
    # Convert eccentricity to integer format (multiply by 10^7, no decimal point)
    ecc_int = int(eccentricity * 10000000)
    
    line2_data = (
        f"2 {satellite_number:5d} "
        f"{inclination_deg:8.4f} "
        f"{raan_deg:8.4f} "
        f"{ecc_int:07d} "
        f"{arg_perigee_deg:8.4f} "
        f"{mean_anomaly_deg:8.4f} "
        f"{mean_motion_rev_per_day:11.8f}"
        f"{revolution_number:5d}"
    )
    
    # Calculate checksum for line 2 (must be exactly 68 chars before checksum)
    line2_68 = line2_data[:68]
    checksum2 = calculate_tle_checksum(line2_68)
    line2 = line2_68 + str(checksum2)
    
    print(f"Generated mock TLE:")
    print(f"Name: {sat_name.strip()}")
    print(f"Inclination: {inclination_deg:.4f}°")
    print(f"RAAN: {raan_deg:.4f}°") 
    print(f"Mean Motion: {mean_motion_rev_per_day:.8f} rev/day")
    print(f"Mean Anomaly: {mean_anomaly_deg:.4f}°")
    print(f"Epoch: {epoch_dt.strftime('%Y-%m-%d %H:%M:%S')} UTC")
    
    return sat_name.strip(), line1, line2


def calculate_tle_checksum(line):
    """
    Calculate TLE checksum according to standard algorithm.
    Sum of all digits plus 1 for each '-' character, modulo 10.
    """
    total = 0
    for char in line[:68]:  # Only first 68 characters count toward checksum
        if char.isdigit():
            total += int(char)
        elif char == '-':
            total += 1
    return total % 10


def format_tle_scientific_notation(value, width=8):
    """
    Format a number in TLE's implied scientific notation format.
    E.g., 0.30199e-3 becomes " 30199-3"
    """
    if value == 0.0:
        return " 00000-0"
    
    # Convert to scientific notation
    exp = int(np.floor(np.log10(abs(value))))
    mantissa = value / (10.0 ** exp)
    
    # Format mantissa (remove decimal point and leading zero)
    if mantissa < 0:
        sign = "-"
        mantissa = abs(mantissa)
    else:
        sign = " "
    
    # Convert mantissa to integer representation
    mantissa_str = f"{mantissa:.5f}"[2:7]  # Take 5 digits after decimal
    
    # Format exponent with sign
    if exp < 0:
        exp_str = f"{sign}{mantissa_str}{abs(exp)}"
    else:
        exp_str = f"{sign}{mantissa_str}+{exp}"
    
    return exp_str.rjust(width)