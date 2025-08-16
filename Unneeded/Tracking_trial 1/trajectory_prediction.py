import numpy as np
from scipy.interpolate import CubicSpline
import time
from constants import *

class TrajectoryPredictor:
    def __init__(self):
        self.points = []  # List of [distance, azimuth, elevation, timestamp]
        self.spline_az = None
        self.spline_el = None
        self.last_update_time = None
        
    def add_point(self, distance, azimuth, elevation):
        current_time = time.time()
        if not self.points:
            self.last_update_time = current_time
            
        self.points.append([distance, azimuth, elevation, current_time])
        
        if len(self.points) >= 3:
            self._update_trajectory()
            
    def _update_trajectory(self):
        """Updates the trajectory prediction based on collected points"""
        points_array = np.array(self.points)
        timestamps = points_array[:, 3] - points_array[0, 3]  # Relative time
        
        # Create splines for azimuth and elevation vs time
        self.spline_az = CubicSpline(timestamps, points_array[:, 1])
        self.spline_el = CubicSpline(timestamps, points_array[:, 2])
        
    def predict_position(self, time_ahead=0.5):
        """Predicts position time_ahead seconds into the future"""
        if self.spline_az is None or self.spline_el is None:
            return None, None
            
        current_time = time.time()
        relative_time = current_time - self.points[0][3] + time_ahead
        
        try:
            predicted_az = float(self.spline_az(relative_time))
            predicted_el = float(self.spline_el(relative_time))
            
            # Normalize angles
            predicted_az = predicted_az % 360
            predicted_el = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, predicted_el))
            
            return predicted_az, predicted_el
        except ValueError:
            return None, None
            
    def get_tracking_window(self):
        """Returns the recommended tracking window size based on prediction confidence"""
        if len(self.points) < 3:
            return DEFAULT_SWEEP_RANGE, DEFAULT_ELEVATION_RANGE
            
        # Calculate position variance to determine tracking window
        az_std = np.std([p[1] for p in self.points[-3:]])
        el_std = np.std([p[2] for p in self.points[-3:]])
        
        # Adjust window based on variance (larger variance = larger window)
        az_window = max(MIN_SWEEP_RANGE, min(DEFAULT_SWEEP_RANGE, az_std * 4))
        el_window = max(MIN_ELEVATION_RANGE, min(DEFAULT_ELEVATION_RANGE, el_std * 4))
        
        return az_window, el_window
