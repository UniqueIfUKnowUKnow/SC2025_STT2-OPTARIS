import math
import numpy as np

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


def build_plane_basis_aligned(n_hat, unit_vectors, azimuth_trend):
    """
    Build plane basis vectors aligned with motion direction.
    
    Parameters:
    n_hat: (3,) plane normal vector
    unit_vectors: (3, N) array of unit vectors
    azimuth_trend: average azimuth change (positive = increasing azimuth)
    
    Returns:
    C, S: basis vectors where C is aligned with motion direction
    """
    # Calculate the motion direction by averaging unit vector differences
    if unit_vectors.shape[1] > 1:
        # Calculate differences between consecutive unit vectors
        motion_vectors = np.diff(unit_vectors, axis=1)
        # Average motion direction
        avg_motion = np.mean(motion_vectors, axis=1)
        # Project onto the plane
        motion_in_plane = avg_motion - np.dot(avg_motion, n_hat) * n_hat
        
        if np.linalg.norm(motion_in_plane) > 1e-6:
            # Use motion direction as primary basis vector
            C = motion_in_plane / np.linalg.norm(motion_in_plane)
        else:
            # Fallback: use first point projected onto plane
            u_first = unit_vectors[:, 0]
            u_plane = u_first - np.dot(n_hat, u_first) * n_hat
            C = u_plane / np.linalg.norm(u_plane)
    else:
        # Single point: use it as reference
        u_first = unit_vectors[:, 0]
        u_plane = u_first - np.dot(n_hat, u_first) * n_hat
        C = u_plane / np.linalg.norm(u_plane)
    
    # Ensure C points in direction of increasing azimuth if azimuth is increasing
    if azimuth_trend > 0:
        # Check if C is pointing in the right direction by testing against known motion
        # The cross product n_hat × C gives the perpendicular direction in the plane
        S_candidate = np.cross(n_hat, C)
        
        # Test orientation: if azimuth increases, phase should increase
        # For small angles, phase ≈ azimuth, so C should have positive x or y component
        # depending on the coordinate system
        if unit_vectors.shape[1] > 1:
            # Calculate actual phase change using the current basis
            phases_test = []
            for i in range(unit_vectors.shape[1]):
                u = unit_vectors[:, i]
                phase = np.arctan2(np.dot(u, S_candidate), np.dot(u, C))
                phases_test.append(phase)
            
            phase_trend = np.mean(np.diff(phases_test))
            
            # If phase trend and azimuth trend have opposite signs, flip the basis
            if (phase_trend > 0) != (azimuth_trend > 0):
                C = -C
    
    # Generate perpendicular vector
    S = np.cross(n_hat, C)
    
    return C, S


def build_plane_basis(n_hat, u_ref):
    """
    Original function for backward compatibility.
    """
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

def debug_coordinate_system(unit_vectors, n_hat, cos_base, sin_base, first_scan_pos):
    """
    Debug function to print coordinate system information.
    """
    print("\n=== COORDINATE SYSTEM DEBUG ===")
    print(f"Plane normal (n_hat): [{n_hat[0]:.4f}, {n_hat[1]:.4f}, {n_hat[2]:.4f}]")
    print(f"Cosine basis (C): [{cos_base[0]:.4f}, {cos_base[1]:.4f}, {cos_base[2]:.4f}]")
    print(f"Sine basis (S): [{sin_base[0]:.4f}, {sin_base[1]:.4f}, {sin_base[2]:.4f}]")
    
    # Verify orthogonality
    dot_nc = np.dot(n_hat, cos_base)
    dot_ns = np.dot(n_hat, sin_base)
    dot_cs = np.dot(cos_base, sin_base)
    print(f"Orthogonality check - n·C: {dot_nc:.6f}, n·S: {dot_ns:.6f}, C·S: {dot_cs:.6f}")
    
    # Check azimuth trend
    azimuth_values = first_scan_pos[:, 1]
    azimuth_trend = np.mean(np.diff(azimuth_values))
    print(f"Azimuth trend: {azimuth_trend:.2f} deg/measurement")
    
    # Calculate phases and check trend
    phases = phase_from_unit(unit_vectors, cos_base, sin_base)
    phase_trend = np.mean(np.diff(phases))
    print(f"Phase trend: {np.degrees(phase_trend):.2f} deg/measurement")
    
    # Check if trends align
    trends_aligned = (azimuth_trend > 0) == (phase_trend > 0)
    print(f"Trends aligned: {trends_aligned}")
    
    return trends_aligned