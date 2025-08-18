import math
import numpy as np

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

def degrees_to_radians(dat_array):
    """
    Convert distance/azimuth/tilt array from degrees to radians.
    
    Parameters:
    dat_array (np.ndarray): Nx3 array where columns are [distance, azimuth, tilt]
                           Distance is kept unchanged, azimuth and tilt converted from degrees to radians
    
    Returns:
    np.ndarray: Nx3 array with [distance, azimuth_rad, tilt_rad]
    """
    # Make a copy to avoid modifying the original array
    result = dat_array.copy()
    
    # Convert azimuth (column 1) and tilt (column 2) from degrees to radians
    # Keep distance (column 0) unchanged
    result[:, 1] = np.deg2rad(dat_array[:, 1])  # azimuth
    result[:, 2] = np.deg2rad(dat_array[:, 2])  # tilt
    
    return result


#returns unit vectors Ux, Uy, Uz
def angles_to_unit(theta, phi):
    """
    Convert spherical angles to unit vectors.
    
    Parameters:
    theta: array-like, azimuthal angles
    phi: array-like, polar angles (elevation from xy-plane)
    
    Returns:
    array of shape (3, N) where N is number of input points
    Each column is [Ux, Uy, Uz] for corresponding theta, phi
    """
    theta = np.asarray(theta)
    phi = np.asarray(phi)
    
    ce = np.cos(phi)
    return np.array([ce*np.cos(theta), ce*np.sin(theta), np.sin(phi)])

def unit_to_angles(u):
    theta = np.arctan2(u[1], u[0])  # az
    phi = np.arcsin(np.clip(u[2], -1.0, 1.0))  # el
    if theta < 0: theta += 2*np.pi
    return theta, phi


def fit_plane_svd(U):
    """
    U: Nx3 array, rows are unit vectors u_i^T.
    Returns:
      n_hat: (3,) unit normal to the best-fit plane through the origin
      sing_vals: (3,) singular values (descending)
      Vt: 3x3 right singular vectors (rows)
    """
    # U = W Σ V^T
    # Smallest singular value's right-singular vector is plane normal
    W, S, Vt = np.linalg.svd(U, full_matrices=False)
    n_hat = Vt[-1, :]  # row corresponding to smallest singular value
    n_hat = n_hat / np.linalg.norm(n_hat)
    return n_hat, S, Vt


def plane_fit_quality(n_hat, U):
    """
    Returns:
      rms: root-mean-square distance to plane (dimensionless ~ radians for small angles)
      max_abs: worst absolute distance
    """
    d = U @ n_hat  # shape (N,)
    rms = float(np.sqrt(np.mean(d**2)))
    max_abs = float(np.max(np.abs(d)))
    return rms, max_abs

#orthagonal basis inside the plane
def build_plane_basis(n_hat, u_ref):
    u_plane = u_ref - np.dot(n_hat, u_ref) * n_hat
    C = u_plane / np.linalg.norm(u_plane)
    S = np.cross(n_hat, C)
    return C, S


def phase_from_unit(u, C, S):
    """
    Calculate phase from unit vectors and reference vectors C, S.
    
    Parameters:
    u: array of shape (3, N) - unit vectors
    C, S: arrays of shape (3,) or (3, N) - reference vectors
    
    Returns:
    array of phases for each unit vector
    """
    u = np.asarray(u)
    C = np.asarray(C)
    S = np.asarray(S)
    
    # Use np.sum with axis=0 to handle dot products along the vector dimension
    if u.ndim == 1:
        # Single vector case
        return np.arctan2(np.dot(u, S), np.dot(u, C))
    else:
        # Multiple vectors case - u should be shape (3, N)
        if C.ndim == 1:
            C = C[:, np.newaxis]  # Broadcast C to (3, 1)
        if S.ndim == 1:
            S = S[:, np.newaxis]  # Broadcast S to (3, 1)
        
        dot_S = np.sum(u * S, axis=0)  # Sum along vector dimension
        dot_C = np.sum(u * C, axis=0)
        return np.arctan2(dot_S, dot_C)

def wrap_to_pi(a):
    """
    Wrap angles to [-π, π] range.
    Works with scalars or arrays.
    """
    return (np.asarray(a) + np.pi) % (2*np.pi) - np.pi

def unwrap_phases(s_list):
    out = [s_list[0]]
    for k in range(1, len(s_list)):
        ds = wrap_to_pi(s_list[k] - s_list[k-1])
        out.append(out[-1] + ds)
    return np.array(out)