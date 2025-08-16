
import numpy as np
import math

# =========================
# ConicSection Class
# =========================
class ConicSection:
    """
    Represents a general conic section (ellipse, parabola, hyperbola, etc.)
    and provides methods for fitting a conic to points and predicting values.
    """
    def __init__(self, coeffs):
        """
        Initialize the conic section with coefficients [A, B, C, D, E, F] for the general conic equation:
            Ax^2 + Bxy + Cy^2 + Dx + Ey + F = 0
        Args:
            coeffs (array-like): List or array of 6 coefficients.
        """
        self.coeffs = coeffs

    @staticmethod
    def fit_from_points(points):
        """
        Fit a conic section (ideally an ellipse) to a set of points using least squares.
        Uses the design matrix method and SVD to solve for the best-fit coefficients.
        Args:
            points (np.ndarray): Nx2 array of (x, y) points.
        Returns:
            ConicSection: Fitted conic section object.
        """
        x = points[:, 0]
        y = points[:, 1]
        # Build the design matrix for the conic equation
        # Each row: [x^2, xy, y^2, x, y, 1]
        D = np.vstack([x**2, x*y, y**2, x, y, np.ones(len(x))]).T
        # Use SVD to solve D @ coeffs = 0 (least squares solution)
        U, S, V = np.linalg.svd(D, full_matrices=False)
        coeffs = V[-1, :]  # The solution is the last row of V
        return ConicSection(coeffs)

    def is_ellipse(self):
        """
        Check if the fitted conic is an ellipse using the discriminant:
            B^2 - 4AC < 0  => ellipse
        Returns:
            bool: True if the conic is an ellipse, False otherwise.
        """
        A, B, C = self.coeffs[0], self.coeffs[1], self.coeffs[2]
        return (B**2 - 4*A*C) < 0

    def predict_distance(self, angle_deg):
        """
        Predict the distance (radius) at a given angle (in degrees) for the ellipse.
        This is done by substituting x = r*cos(a), y = r*sin(a) into the conic equation
        and solving for r (distance) at the given angle.
        Args:
            angle_deg (float): Angle in degrees.
        Returns:
            float or None: Predicted distance, or None if no real solution exists.
        """
        angle_rad = math.radians(angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        A, B, C, D, E, F = self.coeffs
        # Substitute x = r*cos(a), y = r*sin(a) into the conic equation
        # The equation becomes: a*r^2 + b*r + c = 0
        a = A*cos_a**2 + B*cos_a*sin_a + C*sin_a**2
        b = D*cos_a + E*sin_a
        c = F
        discriminant = b**2 - 4*a*c
        # If discriminant is negative or a is too small, no real solution
        if discriminant < 0 or abs(a) < 1e-8:
            return None
        # Use the positive root (forward direction)
        r = (-b + math.sqrt(discriminant)) / (2*a)
        return r

# =========================
# Trajectory Prediction Function
# =========================
def predict_ellipse_trajectory(detected_points_polar):
    """
    Predicts the next point on a trajectory by fitting an ellipse to five detected points.
    Steps:
      1. Convert polar coordinates (angle, distance) to Cartesian (x, y).
      2. Fit an ellipse to the points using least squares.
      3. Predict the next point by evaluating the ellipse at the next angle.
    Args:
        detected_points_polar (list): A list of 5 tuples, each containing (angle, distance).
                                      Angle is in degrees, distance is in cm.
    Returns:
        tuple: (predicted_angle, predicted_distance) for the next point, or (None, None) if failed.
    """
    if len(detected_points_polar) < 5:
        print("Error: At least 5 points are required to fit an ellipse.")
        return None, None

    # Step 1: Convert polar coordinates to Cartesian coordinates
    # x = r*cos(angle), y = r*sin(angle)
    cartesian_points = np.array([
        (dist * math.cos(math.radians(angle)), dist * math.sin(math.radians(angle)))
        for angle, dist in detected_points_polar
    ])

    # Step 2: Fit an ellipse to the Cartesian points
    ellipse = ConicSection.fit_from_points(cartesian_points)
    if not ellipse.is_ellipse():
        print("Warning: The fitted conic is not an ellipse.")
        return None, None

    # Step 3: Predict the next point along the trajectory
    last_angle = detected_points_polar[-1][0]
    PREDICTION_STEP_ANGLE = 2.0  # degrees to step forward
    next_angle = last_angle + PREDICTION_STEP_ANGLE
    predicted_distance = ellipse.predict_distance(next_angle)
    if predicted_distance is None:
        print("Prediction failed: No real solution for the next point.")
        return None, None
    return next_angle, predicted_distance


