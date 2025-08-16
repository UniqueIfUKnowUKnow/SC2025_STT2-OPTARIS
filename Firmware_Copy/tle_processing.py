from dataclasses import dataclass

def _implied_scientific_to_float(s: str) -> float:
    """
    Convert TLE's implied scientific notation (e.g., ' 00000-0', ' 30199-3')
    into a Python float.
    Rules:
      - Strip spaces.
      - Insert a decimal point before the first (sign or digit) and
        interpret the trailing sign+exponent (e.g., -0, -3).
      - Examples:
          ' 00000-0' -> 0.0
          ' 30199-3' -> 0.30199e-3
          '-11606-4' -> -0.11606e-4
    """
    s = s.strip()
    if not s:
        return 0.0
    # Split mantissa and exponent: last two chars are sign+digit(s) of exponent
    # TLE uses forms like "xxxxx-y" or "xxxxx+y"
    mantissa = s[:-2]
    exp_sign = s[-2]
    exp = s[-1]
    # Some sets have two-digit exponents; handle that
    # If second to last char is + or -, search back for that sign.
    for i in range(len(s)-1, -1, -1):
        if s[i] in "+-":
            mantissa = s[:i]
            exp = s[i+1:]
            exp_sign = s[i]
            break
    mantissa = mantissa.strip()
    if not mantissa:
        m = 0.0
    else:
        # Insert decimal before mantissa
        if mantissa[0] in "+-":
            sign = mantissa[0]
            digits = mantissa[1:]
            m = float(f"{sign}0.{digits}")
        else:
            m = float(f"0.{mantissa}")
    e = int(exp) if exp else 0
    if exp_sign == "-":
        e = -e
    return m * (10.0 ** e)

def _tle_checksum_ok(line: str) -> bool:
    """
    TLE checksum: sum of all digits plus 1 for each '-' character,
    modulo 10, must equal the last character of the line.
    """
    total = 0
    for ch in line[:68]:  # exclude the checksum char at col 69 (index 68)
        if ch.isdigit():
            total += int(ch)
        elif ch == "-":
            total += 1
    try:
        checksum = int(line[68])
    except (IndexError, ValueError):
        return False
    return (total % 10) == checksum

@dataclass
class ParsedTLE:
    name: str | None
    satnum: int
    classification: str
    intl_designator: str
    epoch_year: int           # four-digit year
    epoch_day: float          # day-of-year with fraction
    mean_motion_dot: float    # rev/day^2
    mean_motion_ddot: float   # rev/day^3
    bstar: float              # 1/Earth radii
    ephemeris_type: int
    element_set_number: int
    inclination_deg: float
    raan_deg: float
    eccentricity: float
    arg_perigee_deg: float
    mean_anomaly_deg: float
    mean_motion_rev_per_day: float
    rev_number_at_epoch: int
    line1_checksum_ok: bool
    line2_checksum_ok: bool

def parse_tle(tle: list[str] | tuple[str, ...]) -> dict:
    """
    Parse a TLE provided as [name?, line1, line2] into a dictionary of elements.
    Accepts either (name, line1, line2) or just (line1, line2).
    """
    if len(tle) == 3:
        name, line1, line2 = tle
    elif len(tle) == 2:
        name = None
        line1, line2 = tle
    else:
        raise ValueError("TLE must be a list/tuple of length 2 or 3.")

    # Fixed-width slices use 0-based, end-exclusive indices
    satnum = int(line1[2:7])
    classification = line1[7].strip() or "U"
    intl_designator = (line1[9:11] + line1[11:14] + line1[14:17]).strip()

    # Epoch: year (2-digit) and day-of-year with fraction
    epoch_year_2 = int(line1[18:20])
    epoch_year = 1900 + epoch_year_2 if epoch_year_2 >= 57 else 2000 + epoch_year_2
    epoch_day = float(line1[20:32])

    mean_motion_dot = float(line1[33:43])
    mean_motion_ddot = _implied_scientific_to_float(line1[44:52])
    bstar = _implied_scientific_to_float(line1[53:61])

    ephemeris_type = int(line1[62])
    element_set_number = int(line1[64:68])

    # Line 2 orbital elements
    inclination_deg = float(line2[8:16])
    raan_deg = float(line2[17:25])
    eccentricity = float("0." + line2[26:33].strip())
    arg_perigee_deg = float(line2[34:42])
    mean_anomaly_deg = float(line2[43:51])
    mean_motion_rev_per_day = float(line2[52:63])
    rev_number_at_epoch = int(line2[63:68])

    parsed = ParsedTLE(
        name=name,
        satnum=satnum,
        classification=classification,
        intl_designator=intl_designator,
        epoch_year=epoch_year,
        epoch_day=epoch_day,
        mean_motion_dot=mean_motion_dot,
        mean_motion_ddot=mean_motion_ddot,
        bstar=bstar,
        ephemeris_type=ephemeris_type,
        element_set_number=element_set_number,
        inclination_deg=inclination_deg,
        raan_deg=raan_deg,
        eccentricity=eccentricity,
        arg_perigee_deg=arg_perigee_deg,
        mean_anomaly_deg=mean_anomaly_deg,
        mean_motion_rev_per_day=mean_motion_rev_per_day,
        rev_number_at_epoch=rev_number_at_epoch,
        line1_checksum_ok=_tle_checksum_ok(line1),
        line2_checksum_ok=_tle_checksum_ok(line2),
    )

    # Return as a plain dict for easy access / JSON
    return parsed.__dict__

# -----------------------
# Example usage:



# Access like:
# elements["inclination_deg"], elements["mean_motion_rev_per_day"], elements["bstar"], ...
