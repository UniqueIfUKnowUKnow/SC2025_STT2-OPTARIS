import numpy as np
from scipy.linalg import cholesky

class UnscentedKalmanFilter:
    def __init__(self, n_states=6):
        """
        Initialize UKF for 3D tracking (position and velocity)
        States: [x, y, z, vx, vy, vz]
        """
        self.n = n_states
        self.n_sigma = 2 * n_states + 1
        
        # UKF parameters
        self.alpha = 0.1  # Spread of sigma points
        self.beta = 2.0   # Prior knowledge of state distribution (2 for Gaussian)
        self.kappa = 0.0  # Secondary scaling parameter
        self.lambda_ = self.alpha**2 * (self.n + self.kappa) - self.n
        
        # Weights for mean and covariance
        self.weights_m = np.zeros(self.n_sigma)
        self.weights_c = np.zeros(self.n_sigma)
        self._compute_weights()
        
        # State and covariance initialization
        self.x = np.zeros(self.n)  # State vector [x, y, z, vx, vy, vz]
        self.P = np.eye(self.n)    # State covariance
        
        # Process noise (constant velocity model)
        self.Q = np.diag([0.1, 0.1, 0.1, 0.2, 0.2, 0.2])  # Adjust these values based on drone dynamics
        
        # Measurement noise
        self.R = np.diag([0.1, 0.1, 0.1])  # Adjust based on LiDAR accuracy
        
        self.dt = 0.1  # Time step (will be updated in predict)
        
    def _compute_weights(self):
        """Compute weights for the mean and covariance"""
        self.weights_m[0] = self.lambda_ / (self.n + self.lambda_)
        self.weights_c[0] = self.lambda_ / (self.n + self.lambda_) + (1 - self.alpha**2 + self.beta)
        
        for i in range(1, self.n_sigma):
            self.weights_m[i] = 1.0 / (2 * (self.n + self.lambda_))
            self.weights_c[i] = self.weights_m[i]
    
    def _generate_sigma_points(self):
        """Generate sigma points based on current state and covariance"""
        # Calculate square root of (n + λ)P
        L = cholesky((self.n + self.lambda_) * self.P)
        
        # Initialize sigma points matrix
        sigma_points = np.zeros((self.n_sigma, self.n))
        sigma_points[0] = self.x
        
        for i in range(self.n):
            sigma_points[i + 1] = self.x + L[i]
            sigma_points[i + 1 + self.n] = self.x - L[i]
            
        return sigma_points
    
    def _state_transition(self, sigma_point, dt):
        """
        Implement constant velocity model
        x(t+1) = x(t) + v(t)*dt
        v(t+1) = v(t)
        """
        next_state = sigma_point.copy()
        # Update positions using velocities
        next_state[0] += sigma_point[3] * dt  # x += vx*dt
        next_state[1] += sigma_point[4] * dt  # y += vy*dt
        next_state[2] += sigma_point[5] * dt  # z += vz*dt
        return next_state
    
    def predict(self, dt):
        """Predict next state using UKF prediction step"""
        self.dt = dt
        
        # Generate sigma points
        sigma_points = self._generate_sigma_points()
        
        # Propagate sigma points through state transition function
        propagated_sigmas = np.zeros_like(sigma_points)
        for i in range(self.n_sigma):
            propagated_sigmas[i] = self._state_transition(sigma_points[i], dt)
        
        # Compute predicted mean
        self.x = np.zeros(self.n)
        for i in range(self.n_sigma):
            self.x += self.weights_m[i] * propagated_sigmas[i]
        
        # Compute predicted covariance
        self.P = np.zeros((self.n, self.n))
        for i in range(self.n_sigma):
            diff = propagated_sigmas[i] - self.x
            self.P += self.weights_c[i] * np.outer(diff, diff)
        
        # Add process noise
        self.P += self.Q * dt
        
        return self.x, self.P
    
    def update(self, measurement):
        """
        Update state using UKF update step
        measurement: [x, y, z] position from LiDAR
        """
        # Generate sigma points
        sigma_points = self._generate_sigma_points()
        
        # Measurement prediction
        predicted_measurement = sigma_points[:, :3]  # Only position components
        
        # Predicted measurement mean
        z_pred = np.zeros(3)
        for i in range(self.n_sigma):
            z_pred += self.weights_m[i] * predicted_measurement[i]
        
        # Measurement covariance
        S = np.zeros((3, 3))
        Pxz = np.zeros((self.n, 3))
        
        for i in range(self.n_sigma):
            diff_z = predicted_measurement[i] - z_pred
            diff_x = sigma_points[i] - self.x
            S += self.weights_c[i] * np.outer(diff_z, diff_z)
            Pxz += self.weights_c[i] * np.outer(diff_x, diff_z)
        
        S += self.R
        
        # Kalman gain
        K = Pxz @ np.linalg.inv(S)
        
        # Update state and covariance
        measurement_residual = measurement - z_pred
        self.x += K @ measurement_residual
        self.P -= K @ S @ K.T
        
        return self.x, self.P
    
    def get_prediction(self, dt):
        """Get prediction for visualization without updating state"""
        x_pred = self.x.copy()
        x_pred[0:3] += x_pred[3:6] * dt
        return x_pred[:3]  # Return predicted position
