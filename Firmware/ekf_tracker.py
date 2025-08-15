# ekf_tracker.py
import numpy as np
import math
from datetime import datetime
from coordinate_transfer import xyz_to_polar

class DroneEKF:
    """
    Extended Kalman Filter for tracking drone position and velocity in 3D space.
    
    State vector: [x, y, z, vx, vy, vz] where:
    - x, y, z: position in meters (converted from LiDAR coordinate system)
    - vx, vy, vz: velocity in m/s
    """
    
    def __init__(self, initial_position=None, process_noise=0.1, measurement_noise=1.0):
        """
        Initialize the Extended Kalman Filter.
        
        Args:
            initial_position: [x, y, z] initial position in meters, if known
            process_noise: Process noise standard deviation
            measurement_noise: Measurement noise standard deviation (in meters)
        """
        # State dimension (6: position + velocity)
        self.n = 6
        
        # Initialize state vector [x, y, z, vx, vy, vz]
        if initial_position is not None:
            self.x = np.array([initial_position[0], initial_position[1], initial_position[2], 
                              0.0, 0.0, 0.0])
        else:
            self.x = np.zeros(6)
        
        # Initialize covariance matrix (high uncertainty initially)
        self.P = np.eye(6) * 100.0
        
        # Process noise covariance matrix
        self.Q = np.eye(6) * process_noise**2
        self.Q[3:6, 3:6] *= 0.1  # Lower process noise for velocity
        
        # Measurement noise covariance (3x3 for position measurements)
        self.R = np.eye(3) * measurement_noise**2
        
        # Store measurements for trajectory analysis
        self.measurements = []
        self.timestamps = []
        self.predicted_states = []
        self.last_update_time = None
        
        print("Extended Kalman Filter initialized for drone tracking")
    
    def spherical_to_cartesian_meters(self, distance_cm, azimuth_deg, elevation_deg):
        """
        Convert spherical coordinates to Cartesian coordinates in meters.
        
        Args:
            distance_cm: Distance in centimeters
            azimuth_deg: Azimuth angle in degrees
            elevation_deg: Elevation angle in degrees
            
        Returns:
            tuple: (x, y, z) in meters
        """
        # Convert to meters
        distance_m = distance_cm / 100.0
        
        # Convert degrees to radians
        az_rad = math.radians(azimuth_deg)
        el_rad = math.radians(elevation_deg)
        
        # Convert to Cartesian coordinates
        x = distance_m * math.cos(el_rad) * math.sin(az_rad)
        y = distance_m * math.cos(el_rad) * math.cos(az_rad)
        z = distance_m * math.sin(el_rad)
        
        return x, y, z
    
    def predict(self, dt):
        """
        Prediction step of the Kalman filter.
        
        Args:
            dt: Time step in seconds
        """
        # State transition matrix (constant velocity model)
        F = np.eye(6)
        F[0, 3] = dt  # x = x + vx*dt
        F[1, 4] = dt  # y = y + vy*dt
        F[2, 5] = dt  # z = z + vz*dt
        
        # Predict state
        self.x = F @ self.x
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, measurement, timestamp):
        """
        Update step of the Kalman filter with a new measurement.
        
        Args:
            measurement: [distance_cm, azimuth_deg, elevation_deg] from LiDAR
            timestamp: Timestamp of the measurement
        """
        # Convert measurement to Cartesian coordinates
        distance_cm, azimuth_deg, elevation_deg = measurement
        x_meas, y_meas, z_meas = self.spherical_to_cartesian_meters(
            distance_cm, azimuth_deg, elevation_deg
        )
        z_meas = np.array([x_meas, y_meas, z_meas])
        
        # Store raw position for velocity initialization
        if len(self.measurements) == 0:
            # First measurement - initialize position only
            self.x[:3] = z_meas
            self.x[3:6] = 0.0  # Zero velocity initially
            print(f"EKF initialized at position: ({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f})m")
        
        elif len(self.measurements) == 1:
            # Second measurement - calculate initial velocity estimate
            dt = (timestamp - self.last_update_time).total_seconds()
            if dt > 0:
                prev_pos = np.array([self.measurements[0][0], self.measurements[0][1], self.measurements[0][2]])
                prev_x, prev_y, prev_z = self.spherical_to_cartesian_meters(prev_pos[0], prev_pos[1], prev_pos[2])
                prev_pos_cart = np.array([prev_x, prev_y, prev_z])
                
                # Direct velocity calculation from position difference
                velocity_est = (z_meas - prev_pos_cart) / dt
                self.x[:3] = z_meas
                self.x[3:6] = velocity_est
                
                print(f"EKF velocity initialized: ({velocity_est[0]:.2f}, {velocity_est[1]:.2f}, {velocity_est[2]:.2f})m/s")
                print(f"Speed: {np.linalg.norm(velocity_est):.2f} m/s")
        
        else:
            # Third+ measurement - use normal Kalman filter update
            # Calculate time step
            dt = (timestamp - self.last_update_time).total_seconds()
            if dt > 0:
                self.predict(dt)
            
            # Measurement matrix (we measure position, not velocity)
            H = np.zeros((3, 6))
            H[0, 0] = 1  # x
            H[1, 1] = 1  # y
            H[2, 2] = 1  # z
            
            # Innovation (residual)
            h_x = H @ self.x  # Predicted measurement
            y = z_meas - h_x  # Innovation
            
            # Innovation covariance
            S = H @ self.P @ H.T + self.R
            
            # Kalman gain
            K = self.P @ H.T @ np.linalg.inv(S)
            
            # Update state
            self.x = self.x + K @ y
            
            # Update covariance
            I = np.eye(6)
            self.P = (I - K @ H) @ self.P
        
        # Store data for analysis
        self.measurements.append(measurement)
        self.timestamps.append(timestamp)
        self.predicted_states.append(self.x.copy())
        self.last_update_time = timestamp
        
        if len(self.measurements) >= 2:
            print(f"EKF updated: pos=({self.x[0]:.2f}, {self.x[1]:.2f}, {self.x[2]:.2f})m, "
                  f"vel=({self.x[3]:.2f}, {self.x[4]:.2f}, {self.x[5]:.2f})m/s")
    
    def predict_future_position(self, seconds_ahead):
        """
        Predict the drone's position at a specific time in the future.
        
        Args:
            seconds_ahead: Number of seconds into the future
            
        Returns:
            tuple: (x, y, z, vx, vy, vz) predicted state
        """
        # Use current state as starting point
        future_state = self.x.copy()
        
        # Apply constant velocity model
        future_state[0] += self.x[3] * seconds_ahead  # x += vx * dt
        future_state[1] += self.x[4] * seconds_ahead  # y += vy * dt
        future_state[2] += self.x[5] * seconds_ahead  # z += vz * dt
        
        return future_state
    
    def predict_future_spherical(self, seconds_ahead):
        """
        Predict the drone's future position in spherical coordinates.
        
        Args:
            seconds_ahead: Number of seconds into the future
            
        Returns:
            tuple: (distance_cm, azimuth_deg, elevation_deg)
        """
        future_state = self.predict_future_position(seconds_ahead)
        x, y, z = future_state[0], future_state[1], future_state[2]
        
        # Convert back to spherical coordinates
        azimuth_deg, elevation_deg, distance_m = xyz_to_polar(x, y, z)
        distance_cm = distance_m * 100.0
        
        return distance_cm, azimuth_deg, elevation_deg
    
    def get_trajectory_points(self, duration_seconds, time_step=0.5):
        """
        Generate trajectory points for visualization or tracking.
        
        Args:
            duration_seconds: How far into the future to predict
            time_step: Time step between trajectory points
            
        Returns:
            list: List of (time, x, y, z, distance_cm, azimuth_deg, elevation_deg) tuples
        """
        trajectory = []
        current_time = 0.0
        
        while current_time <= duration_seconds:
            future_state = self.predict_future_position(current_time)
            x, y, z = future_state[0], future_state[1], future_state[2]
            
            # Convert to spherical for motor positioning
            azimuth_deg, elevation_deg, distance_m = xyz_to_polar(x, y, z)
            distance_cm = distance_m * 100.0
            
            trajectory.append((current_time, x, y, z, distance_cm, azimuth_deg, elevation_deg))
            current_time += time_step
        
        return trajectory
    
    def validate_velocity_direction(self):
        """
        Check if the velocity direction makes sense based on recent measurements.
        Returns True if velocity seems correct, False if it might be wrong.
        """
        if len(self.measurements) < 3:
            return True  # Not enough data to validate
        
        # Check last 3 measurements to see if motion is consistent
        recent_positions = []
        for i in range(max(0, len(self.measurements) - 3), len(self.measurements)):
            measurement = self.measurements[i]
            x, y, z = self.spherical_to_cartesian_meters(measurement[0], measurement[1], measurement[2])
            recent_positions.append([x, y, z])
        
        if len(recent_positions) >= 3:
            # Calculate velocity from actual position changes
            p1, p2, p3 = recent_positions[-3], recent_positions[-2], recent_positions[-1]
            
            # Time intervals (assuming roughly equal spacing)
            actual_vel_1 = np.array(p2) - np.array(p1)
            actual_vel_2 = np.array(p3) - np.array(p2)
            
            # Check if velocities are in similar direction (dot product > 0)
            if len(actual_vel_1) == 3 and len(actual_vel_2) == 3:
                dot_product = np.dot(actual_vel_1, actual_vel_2)
                magnitudes = np.linalg.norm(actual_vel_1) * np.linalg.norm(actual_vel_2)
                
                if magnitudes > 0:
                    direction_consistency = dot_product / magnitudes
                    return direction_consistency > 0.5  # Velocities should be in similar direction
        
        return True
    
    def recalibrate_velocity(self):
        """
        Recalculate velocity based on the last few measurements if direction seems wrong.
        """
        if len(self.measurements) < 3:
            return
        
        print("Recalibrating velocity based on recent measurements...")
        
        # Use last 3 measurements to get a better velocity estimate
        recent_measurements = self.measurements[-3:]
        recent_timestamps = self.timestamps[-3:]
        
        # Convert to Cartesian
        recent_positions = []
        for measurement in recent_measurements:
            x, y, z = self.spherical_to_cartesian_meters(measurement[0], measurement[1], measurement[2])
            recent_positions.append([x, y, z])
        
        # Calculate average velocity over the recent measurements
        total_displacement = np.array(recent_positions[-1]) - np.array(recent_positions[0])
        total_time = (recent_timestamps[-1] - recent_timestamps[0]).total_seconds()
        
        if total_time > 0:
            new_velocity = total_displacement / total_time
            self.x[3:6] = new_velocity
            
            print(f"Velocity recalibrated to: ({new_velocity[0]:.2f}, {new_velocity[1]:.2f}, {new_velocity[2]:.2f})m/s")
            print(f"New speed: {np.linalg.norm(new_velocity):.2f} m/s")
            
            # Reduce velocity uncertainty since we just recalibrated
            self.P[3:6, 3:6] *= 0.5
    
    def get_tracking_confidence(self):
        """
        Get a confidence measure for the current tracking quality.
        
        Returns:
            float: Confidence score (0-1), higher is better
        """
        # Calculate confidence based on covariance trace (lower trace = higher confidence)
        position_variance = np.trace(self.P[:3, :3])
        velocity_variance = np.trace(self.P[3:6, 3:6])
        
        # Normalize to 0-1 scale (adjust these thresholds based on your system)
        pos_confidence = max(0, 1 - position_variance / 100)
        vel_confidence = max(0, 1 - velocity_variance / 10)
        
        return (pos_confidence + vel_confidence) / 2
    
    def reset_filter(self, initial_position=None):
        """
        Reset the filter state (useful if tracking is lost).
        
        Args:
            initial_position: New initial position [x, y, z] in meters
        """
        if initial_position is not None:
            self.x = np.array([initial_position[0], initial_position[1], initial_position[2], 
                              0.0, 0.0, 0.0])
        else:
            self.x = np.zeros(6)
        
        self.P = np.eye(6) * 100.0
        self.measurements.clear()
        self.timestamps.clear()
        self.predicted_states.clear()
        self.last_update_time = None
        
        print("EKF filter reset")
    
    def get_measurement_count(self):
        """
        Get the number of measurements processed.
        
        Returns:
            int: Number of measurements
        """
        return len(self.measurements)
    
    def estimate_intercept_time(self, target_speed_mps=None):
        """
        Estimate when the drone will be closest to the origin (intercept time).
        
        Args:
            target_speed_mps: If provided, assumes drone maintains this speed
            
        Returns:
            float: Time in seconds until closest approach (negative if already past)
        """
        # Current position and velocity
        pos = self.x[:3]
        vel = self.x[3:6]
        
        if target_speed_mps is not None:
            # Normalize velocity to target speed
            current_speed = np.linalg.norm(vel)
            if current_speed > 0:
                vel = vel * (target_speed_mps / current_speed)
        
        # Calculate time of closest approach using calculus
        # Minimize |pos + vel*t|² with respect to t
        # d/dt[|pos + vel*t|²] = 2(pos + vel*t)·vel = 0
        # 2pos·vel + 2vel·vel*t = 0
        # t = -pos·vel / |vel|²
        
        vel_magnitude_sq = np.dot(vel, vel)
        if vel_magnitude_sq < 1e-6:  # Nearly stationary
            return float('inf')
        
        t_intercept = -np.dot(pos, vel) / vel_magnitude_sq
        return t_intercept


# Convenience functions for integration with existing code
def create_drone_tracker(initial_detection=None, process_noise=0.1, measurement_noise=1.0):
    """
    Create a new drone tracker instance.
    
    Args:
        initial_detection: [distance_cm, azimuth_deg, elevation_deg] if available
        process_noise: Process noise parameter
        measurement_noise: Measurement noise parameter
        
    Returns:
        DroneEKF: Initialized tracker
    """
    if initial_detection is not None:
        distance_cm, azimuth_deg, elevation_deg = initial_detection
        tracker = DroneEKF(process_noise=process_noise, measurement_noise=measurement_noise)
        x, y, z = tracker.spherical_to_cartesian_meters(distance_cm, azimuth_deg, elevation_deg)
        tracker.x[:3] = [x, y, z]
    else:
        tracker = DroneEKF(process_noise=process_noise, measurement_noise=measurement_noise)
    
    return tracker

def update_tracker_with_anomaly_data(tracker, anomaly_averaged_coords):
    """
    Update tracker with averaged anomaly coordinate data.
    
    Args:
        tracker: DroneEKF instance
        anomaly_averaged_coords: List of averaged anomaly measurements
    """
    for coords_group in anomaly_averaged_coords:
        if len(coords_group) >= 4:  # [distance, azimuth, elevation, timestamp]
            distance, azimuth, elevation, timestamp = coords_group[:4]
            
            # Convert timestamp to datetime if it's a float (Unix timestamp)
            if isinstance(timestamp, (int, float)):
                dt_timestamp = datetime.fromtimestamp(timestamp)
            else:
                dt_timestamp = timestamp
            
            measurement = [distance, azimuth, elevation]
            tracker.update(measurement, dt_timestamp)
    
    return tracker

def get_current_velocity_magnitude(self):
    """Get the magnitude of current velocity vector."""
    return np.linalg.norm(self.x[3:6])

def predict_with_validation(self, seconds_ahead):
    """Predict future position with validation checks."""
    if self.validate_velocity_direction():
        return self.predict_future_position(seconds_ahead)
    else:
        self.recalibrate_velocity()
        return self.predict_future_position(seconds_ahead)