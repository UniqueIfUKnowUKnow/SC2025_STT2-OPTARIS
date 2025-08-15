import numpy as np
import matplotlib.pyplot as plt
import math
from constants import KALMAN_ERROR_RANGE

class DroneTrajectoryKalman:
    def __init__(self, measurement_error_deg=KALMAN_ERROR_RANGE):
        """
        Initialize Kalman filter for drone trajectory tracking in polar coordinates.
        
        Args:
            measurement_error_deg: Measurement error in degrees for azimuth and elevation
        """
        # Convert measurement error to radians
        self.measurement_error_rad = np.radians(measurement_error_deg)
        
        # State vector: [distance, azimuth, elevation, d_dist/dt, d_azim/dt, d_elev/dt]
        self.state_dim = 6
        self.measurement_dim = 3
        
        # Initialize state vector and covariance
        self.x = np.zeros(self.state_dim)  # State vector
        self.P = np.eye(self.state_dim) * 100  # Covariance matrix (high initial uncertainty)
        
        # Process noise covariance (assuming smooth motion)
        self.Q = np.eye(self.state_dim)
        self.Q[0, 0] = 1.0   # distance process noise
        self.Q[1, 1] = 0.01  # azimuth process noise (rad^2)
        self.Q[2, 2] = 0.01  # elevation process noise (rad^2)
        self.Q[3, 3] = 0.5   # distance velocity noise
        self.Q[4, 4] = 0.005 # azimuth velocity noise
        self.Q[5, 5] = 0.005 # elevation velocity noise
        
        # Measurement noise covariance
        self.R = np.eye(self.measurement_dim)
        self.R[0, 0] = 4.0  # distance measurement noise (assuming 2m std)
        self.R[1, 1] = self.measurement_error_rad**2  # azimuth noise
        self.R[2, 2] = self.measurement_error_rad**2  # elevation noise
        
        # Store measurements for trajectory fitting
        self.measurements = []
        self.timestamps = []
        self.predictions = []
        
    def state_transition_matrix(self, dt):
        """Create state transition matrix F for time step dt"""
        F = np.eye(self.state_dim)
        F[0, 3] = dt  # distance += velocity * dt
        F[1, 4] = dt  # azimuth += angular_velocity * dt
        F[2, 5] = dt  # elevation += angular_velocity * dt
        return F
    
    def measurement_matrix(self):
        """Create measurement matrix H"""
        H = np.zeros((self.measurement_dim, self.state_dim))
        H[0, 0] = 1  # measure distance
        H[1, 1] = 1  # measure azimuth
        H[2, 2] = 1  # measure elevation
        return H
    
    def initialize_from_measurements(self, measurements, timestamps):
        """
        Initialize the filter state from initial measurements.
        
        Args:
            measurements: List of [distance, azimuth, elevation] in [m, rad, rad]
            timestamps: List of timestamps corresponding to measurements
        """
        if len(measurements) < 2:
            raise ValueError("Need at least 2 measurements to initialize velocities")
        
        self.measurements = measurements.copy()
        self.timestamps = timestamps.copy()
        
        # Initialize position from first measurement
        self.x[0] = measurements[0][0]  # distance
        self.x[1] = measurements[0][1]  # azimuth
        self.x[2] = measurements[0][2]  # elevation
        
        # Estimate initial velocities from first two measurements
        dt = timestamps[1] - timestamps[0]
        if dt > 0:
            self.x[3] = (measurements[1][0] - measurements[0][0]) / dt  # distance velocity
            self.x[4] = self.angle_difference(measurements[1][1], measurements[0][1]) / dt  # azimuth velocity
            self.x[5] = self.angle_difference(measurements[1][2], measurements[0][2]) / dt  # elevation velocity
        
        # Reduce initial uncertainty for positions we've measured
        self.P[0, 0] = self.R[0, 0]  # distance uncertainty
        self.P[1, 1] = self.R[1, 1]  # azimuth uncertainty
        self.P[2, 2] = self.R[2, 2]  # elevation uncertainty
    
    def angle_difference(self, angle1, angle2):
        """Calculate the shortest angular difference between two angles"""
        diff = angle1 - angle2
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff
    
    def predict(self, dt):
        """Predict next state after time dt"""
        F = self.state_transition_matrix(dt)
        
        # Predict state
        self.x = F @ self.x
        
        # Normalize angles to [-pi, pi]
        self.x[1] = np.arctan2(np.sin(self.x[1]), np.cos(self.x[1]))
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
        
        return self.x.copy()
    
    def update(self, measurement):
        """Update state with new measurement [distance, azimuth, elevation]"""
        z = np.array(measurement)
        H = self.measurement_matrix()
        
        # Innovation
        y = z - H @ self.x
        
        # Handle angle wrapping for azimuth and elevation
        y[1] = self.angle_difference(z[1], self.x[1])
        y[2] = self.angle_difference(z[2], self.x[2])
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Normalize angles
        self.x[1] = np.arctan2(np.sin(self.x[1]), np.cos(self.x[1]))
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))
        
        # Update covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ H) @ self.P
        
        return self.x.copy()
    
    def predict_future_positions(self, future_times):
        """
        Predict positions at future timestamps.
        
        Args:
            future_times: List of future timestamps
            
        Returns:
            List of predicted [distance, azimuth, elevation] measurements
        """
        if not self.timestamps:
            raise ValueError("No measurements available. Initialize first.")
        
        predictions = []
        current_time = self.timestamps[-1]
        
        # Save current state
        saved_state = self.x.copy()
        saved_P = self.P.copy()
        
        for future_time in future_times:
            dt = future_time - current_time
            if dt <= 0:
                raise ValueError("Future time must be after current time")
            
            # Predict forward
            predicted_state = self.predict(dt)
            
            # Extract position prediction
            prediction = [predicted_state[0], predicted_state[1], predicted_state[2]]
            predictions.append(prediction)
            
            # Reset state for next prediction
            self.x = saved_state.copy()
            self.P = saved_P.copy()
            
        return predictions
    
    def update_with_measurement_at_time(self, measurement, measurement_time):
        """
        Update the filter with a measurement taken at a specific time.
        This handles cases where measurements don't arrive at expected times.
        
        Args:
            measurement: [distance, azimuth, elevation] measurement
            measurement_time: Actual timestamp when measurement was taken
        """
        if not self.timestamps:
            raise ValueError("Filter must be initialized first")
        
        current_time = self.timestamps[-1] if self.timestamps else 0
        dt = measurement_time - current_time
        
        if dt < 0:
            print(f"Warning: Measurement time {measurement_time} is before current filter time {current_time}")
            print("This measurement may be out of sequence. Processing anyway with dt=0")
            dt = 0
        
        # Predict forward to measurement time
        if dt > 0:
            self.predict(dt)
        
        # Update with the measurement
        self.update(measurement)
        
        # Update our time tracking
        self.timestamps.append(measurement_time)
        self.measurements.append(measurement)
        
        return self.x.copy()
    
    def predict_and_wait_for_measurement(self, expected_time, timeout_seconds=5.0):
        """
        Predict position at expected time and prepare for measurement update.
        Returns prediction and allows for measurement at different actual time.
        
        Args:
            expected_time: When you expect to take the measurement
            timeout_seconds: Maximum reasonable delay for measurement
            
        Returns:
            predicted_position: [distance, azimuth, elevation] prediction
        """
        if not self.timestamps:
            raise ValueError("Filter must be initialized first")
        
        current_time = self.timestamps[-1]
        dt = expected_time - current_time
        
        if dt <= 0:
            raise ValueError("Expected time must be after current time")
        
        # Save current state (don't modify the filter yet)
        saved_state = self.x.copy()
        saved_P = self.P.copy()
        
        # Predict to expected time
        predicted_state = self.predict(dt)
        prediction = [predicted_state[0], predicted_state[1], predicted_state[2]]
        
        # Restore state - we'll do the actual prediction when measurement arrives
        self.x = saved_state
        self.P = saved_P
        
        return prediction
    
    def process_measurement_sequence(self, measurements, timestamps):
        """
        Process a sequence of measurements through the Kalman filter.
        
        Args:
            measurements: List of [distance, azimuth, elevation] measurements
            timestamps: Corresponding timestamps
        """
        if len(measurements) != len(timestamps):
            raise ValueError("Measurements and timestamps must have same length")
        
        if len(measurements) < 2:
            raise ValueError("Need at least 2 measurements")
        
        # Initialize with first few measurements
        self.initialize_from_measurements(measurements[:2], timestamps[:2])
        
        # Process remaining measurements
        for i in range(2, len(measurements)):
            dt = timestamps[i] - timestamps[i-1]
            
            # Predict
            self.predict(dt)
            
            # Update with measurement
            self.update(measurements[i])
            
            # Store for analysis
            self.predictions.append(self.x[:3].copy())
    
    def get_trajectory_uncertainty(self):
        """Get current position uncertainty (standard deviations)"""
        return np.sqrt(np.diag(self.P)[:3])
    
    def polar_to_cartesian(self, polar_coords):
        """Convert polar coordinates [distance, azimuth, elevation] to Cartesian [x, y, z]"""
        distance, azimuth, elevation = polar_coords
        x = distance * np.cos(elevation) * np.cos(azimuth)
        y = distance * np.cos(elevation) * np.sin(azimuth)
        z = distance * np.sin(elevation)
        return [x, y, z]
    
    def visualize_trajectory(self, measurements, timestamps, future_predictions=None, future_times=None):
        """Visualize the trajectory in 3D"""
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Convert measurements to Cartesian for visualization
        cartesian_measurements = [self.polar_to_cartesian(m) for m in measurements]
        x_meas = [c[0] for c in cartesian_measurements]
        y_meas = [c[1] for c in cartesian_measurements]
        z_meas = [c[2] for c in cartesian_measurements]
        
        # Plot measurements
        ax.scatter(x_meas, y_meas, z_meas, c='red', s=50, label='Measurements', alpha=0.8)
        ax.plot(x_meas, y_meas, z_meas, 'r--', alpha=0.5)
        
        # Plot Kalman filter predictions if available
        if self.predictions:
            cartesian_predictions = [self.polar_to_cartesian(p) for p in self.predictions]
            x_pred = [c[0] for c in cartesian_predictions]
            y_pred = [c[1] for c in cartesian_predictions]
            z_pred = [c[2] for c in cartesian_predictions]
            
            ax.scatter(x_pred, y_pred, z_pred, c='blue', s=30, label='Kalman Predictions', alpha=0.6)
            ax.plot(x_pred, y_pred, z_pred, 'b-', alpha=0.7)
        
        # Plot future predictions
        if future_predictions:
            cartesian_future = [self.polar_to_cartesian(p) for p in future_predictions]
            x_fut = [c[0] for c in cartesian_future]
            y_fut = [c[1] for c in cartesian_future]
            z_fut = [c[2] for c in cartesian_future]
            
            ax.scatter(x_fut, y_fut, z_fut, c='green', s=40, label='Future Predictions', alpha=0.8)
            ax.plot(x_fut, y_fut, z_fut, 'g-', linewidth=2)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        ax.set_title('Drone Trajectory Tracking')
        plt.show()


def example_usage():
    """Example of how to use the DroneTrajectoryKalman class"""
    
    # Generate example measurements (simulating a conic section trajectory)
    # This simulates a drone following an elliptical path
    np.random.seed(42)
    
    timestamps = np.linspace(0, 10, 5)  # 5 measurements over 10 seconds
    true_trajectory = []
    measurements = []
    
    # Generate synthetic conic section trajectory (ellipse in 3D)
    for i, t in enumerate(timestamps):
        # Parametric ellipse with some 3D component
        angle = t * 0.6  # angular progression
        distance = 100 + 20 * np.cos(angle)  # varying distance (ellipse)
        azimuth = angle + 0.1 * np.sin(2 * angle)  # slight perturbation
        elevation = 0.3 * np.sin(angle * 0.8)  # elevation variation
        
        true_pos = [distance, azimuth, elevation]
        true_trajectory.append(true_pos)
        
        # Add measurement noise
        noise_distance = np.random.normal(0, 2)  # 2m std in distance
        noise_azimuth = np.random.normal(0, np.radians(1))  # 1 degree std
        noise_elevation = np.random.normal(0, np.radians(1))  # 1 degree std
        
        measured_pos = [
            distance + noise_distance,
            azimuth + noise_azimuth,
            elevation + noise_elevation
        ]
        measurements.append(measured_pos)
    
    print("Example: Drone Trajectory Tracking with Kalman Filter")
    print("=" * 55)
    
    # Create and initialize Kalman filter
    kf = DroneTrajectoryKalman(measurement_error_deg=1.0)
    
    # Process the measurements
    print(f"Processing {len(measurements)} measurements...")
    kf.process_measurement_sequence(measurements, timestamps)
    
    # Get current state
    current_state = kf.x
    uncertainty = kf.get_trajectory_uncertainty()
    
    print(f"\nCurrent estimated state:")
    print(f"Position: dist={current_state[0]:.1f}m, az={np.degrees(current_state[1]):.1f}°, el={np.degrees(current_state[2]):.1f}°")
    print(f"Velocity: v_dist={current_state[3]:.2f}m/s, v_az={np.degrees(current_state[4]):.2f}°/s, v_el={np.degrees(current_state[5]):.2f}°/s")
    print(f"Position uncertainty (1σ): ±{uncertainty[0]:.1f}m, ±{np.degrees(uncertainty[1]):.1f}°, ±{np.degrees(uncertainty[2]):.1f}°")
    
    # Predict future positions
    future_times = np.linspace(timestamps[-1] + 1, timestamps[-1] + 5, 5)
    future_predictions = kf.predict_future_positions(future_times)
    
    print(f"\nFuture predictions:")
    for i, (t, pred) in enumerate(zip(future_times, future_predictions)):
        print(f"t={t:.1f}s: dist={pred[0]:.1f}m, az={np.degrees(pred[1]):.1f}°, el={np.degrees(pred[2]):.1f}°")
    
    # Visualize results
    print(f"\nGenerating 3D trajectory visualization...")
    kf.visualize_trajectory(measurements, timestamps, future_predictions, future_times)
    
    # Simulate sensor control commands
    print(f"\nSensor control commands for future positions:")
    print("Time\t\tAzimuth\t\tElevation")
    print("-" * 40)
    for t, pred in zip(future_times, future_predictions):
        az_deg = np.degrees(pred[1])
        el_deg = np.degrees(pred[2])
        print(f"{t:.1f}s\t\t{az_deg:.1f}°\t\t{el_deg:.1f}°")
    
    return kf, measurements, timestamps, future_predictions, future_times


# Function to use your own measurement data
def process_your_data(measurements_polar, timestamps):
    """
    Process your own measurement data.
    
    Args:
        measurements_polar: List of [distance, azimuth_deg, elevation_deg] measurements
        timestamps: List of timestamps
    
    Returns:
        Configured Kalman filter and predictions
    """
    # Convert degrees to radians for azimuth and elevation
    measurements_rad = []
    for dist, az_deg, el_deg in measurements_polar:
        measurements_rad.append([dist, np.radians(az_deg), np.radians(el_deg)])
    
    # Create and process with Kalman filter
    kf = DroneTrajectoryKalman(measurement_error_deg=1.0)
    kf.process_measurement_sequence(measurements_rad, timestamps)
    
    return kf


def real_time_tracking_example():
    """
    Example demonstrating real-time tracking with delayed/unexpected measurement times
    """
    print("Real-time Tracking with Variable Measurement Times")
    print("=" * 50)
    
    # Simulate initial setup
    kf = DroneTrajectoryKalman(measurement_error_deg=1.0)
    
    # Initial measurements to get started
    initial_measurements = [
        [100, np.radians(30), np.radians(10)],
        [102, np.radians(32), np.radians(12)]
    ]
    initial_times = [0.0, 1.0]
    
    kf.initialize_from_measurements(initial_measurements, initial_times)
    print("Filter initialized with 2 measurements")
    
    # Simulate real-time tracking cycle
    expected_measurement_times = [2.5, 4.0, 5.5, 7.0]
    actual_measurement_times = [2.7, 3.8, 5.9, 7.3]  # Slightly different actual times
    
    # Simulate actual measurements (with some trajectory evolution)
    actual_measurements = [
        [104, np.radians(34), np.radians(14)],
        [105, np.radians(35), np.radians(15)], 
        [107, np.radians(37), np.radians(16)],
        [108, np.radians(38), np.radians(17)]
    ]
    
    for i, expected_time in enumerate(expected_measurement_times):
        print(f"\n--- Tracking Cycle {i+1} ---")
        
        # Step 1: Predict where to point sensor
        prediction = kf.predict_and_wait_for_measurement(expected_time)
        pred_az_deg = np.degrees(prediction[1])
        pred_el_deg = np.degrees(prediction[2])
        
        print(f"Expected measurement at t={expected_time}s")
        print(f"Pointing sensor to: az={pred_az_deg:.1f}°, el={pred_el_deg:.1f}°")
        
        # Step 2: Sensor takes measurement at slightly different time
        actual_time = actual_measurement_times[i]
        actual_measurement = actual_measurements[i]
        
        print(f"Actual measurement taken at t={actual_time}s (Δt={actual_time-expected_time:.1f}s)")
        
        # Step 3: Update filter with measurement at actual time
        updated_state = kf.update_with_delayed_measurement(actual_measurement, actual_time)
        
        actual_az_deg = np.degrees(actual_measurement[1])
        actual_el_deg = np.degrees(actual_measurement[2])
        
        print(f"Actual measurement: az={actual_az_deg:.1f}°, el={actual_el_deg:.1f}°")
        print(f"Prediction error: az={actual_az_deg-pred_az_deg:.1f}°, el={actual_el_deg-pred_el_deg:.1f}°")
        
        # Show updated state
        uncertainty = kf.get_trajectory_uncertainty()
        print(f"Updated position uncertainty: ±{np.degrees(uncertainty[1]):.1f}° az, ±{np.degrees(uncertainty[2]):.1f}° el")
    
    return kf


if __name__ == "__main__":
    # Run example
    kf, measurements, timestamps, future_predictions, future_times = example_usage()
    
    print("\n" + "="*60)
    # Run real-time tracking example
    rt_kf = real_time_tracking_example()
    
    print(f"\nTo use with your own data:")
    print("=" * 30)
    print("# Your data format: [distance_m, azimuth_deg, elevation_deg]")
    print("# your_measurements = [[100, 45, 15], [105, 50, 12], ...]")
    print("# your_timestamps = [0, 1, 2, ...]")
    print("# kf = process_your_data(your_measurements, your_timestamps)")
    print("# future_times = [10, 11, 12, 13, 14]")
    print("# predictions = kf.predict_future_positions(future_times)")
    print("\n# For real-time tracking:")
    print("# prediction = kf.predict_and_wait_for_measurement(expected_time)")
    print("# # ... point sensor, wait for measurement ...")
    print("# kf.update_with_delayed_measurement(measurement, actual_time)")