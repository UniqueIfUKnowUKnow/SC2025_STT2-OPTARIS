import numpy as np
from scipy.interpolate import interp1d
import time
from constants import *
from move_motors import set_servo_angle, stepper_step
import RPi.GPIO as GPIO

class TrajectoryTracker:
    def __init__(self, pi, lidar_data_queue):
        self.pi = pi
        self.lidar_data_queue = lidar_data_queue
        self.points = []
        self.trajectory = None
        self.current_azimuth = 0
        self.current_elevation = SERVO_SWEEP_START
        self.last_update_time = time.time()
        self.tracking_width = NARROW_SCAN_WIDTH
        self.tracking_center = 0

    def update_trajectory(self, new_point):
        """
        Updates the trajectory prediction with a new detected point.
        Returns True if trajectory was significantly updated.
        """
        self.points.append(new_point)
        if len(self.points) > MAX_POINTS_MEMORY:
            self.points.pop(0)  # Remove oldest point

        # Sort points by timestamp
        sorted_points = sorted(self.points, key=lambda x: x['timestamp'])
        
        if len(sorted_points) >= 3:
            # Extract coordinates and times for interpolation
            times = np.array([p['timestamp'] for p in sorted_points])
            azimuths = np.array([p['azimuth'] for p in sorted_points])
            elevations = np.array([p['elevation'] for p in sorted_points])
            
            # Create interpolation functions
            self.trajectory = {
                'azimuth': interp1d(times, azimuths, fill_value='extrapolate'),
                'elevation': interp1d(times, elevations, fill_value='extrapolate')
            }
            
            # Predict next position
            next_time = time.time() + 1.0  # Predict 1 second ahead
            predicted_az = float(self.trajectory['azimuth'](next_time))
            predicted_el = float(self.trajectory['elevation'](next_time))
            
            # Check if prediction differs significantly from current tracking center
            if abs(predicted_az - self.tracking_center) > TRAJECTORY_UPDATE_THRESHOLD:
                self.tracking_center = predicted_az
                return True

        return False

    def follow_trajectory(self):
        """
        Performs narrow zigzag scanning pattern following the predicted trajectory.
        Yields any new detected points.
        """
        while True:
            if self.trajectory is not None:
                # Predict current position
                current_time = time.time()
                target_az = float(self.trajectory['azimuth'](current_time))
                target_el = float(self.trajectory['elevation'](current_time))
                
                # Calculate scan bounds
                left_bound = target_az - self.tracking_width/2
                right_bound = target_az + self.tracking_width/2
                
                # Adjust elevation
                if abs(target_el - self.current_elevation) > ZIG_ZAG_ELEVATION_INCREMENT:
                    self.current_elevation += ZIG_ZAG_ELEVATION_INCREMENT * (
                        1 if target_el > self.current_elevation else -1
                    )
                    set_servo_angle(self.pi, self.current_elevation)
                
                # Move stepper for scanning
                if self.current_azimuth >= right_bound:
                    GPIO.output(DIR_PIN, GPIO.LOW)  # Move left
                elif self.current_azimuth <= left_bound:
                    GPIO.output(DIR_PIN, GPIO.HIGH)  # Move right
                
                # Move at increased speed
                steps = int((ZIG_ZAG_STEP_SIZE * TRACKING_SPEED_MULTIPLIER / 360.0) * STEPS_PER_REVOLUTION)
                for _ in range(steps):
                    stepper_step()
                    if GPIO.input(DIR_PIN) == GPIO.HIGH:
                        self.current_azimuth += ZIG_ZAG_STEP_SIZE * TRACKING_SPEED_MULTIPLIER
                    else:
                        self.current_azimuth -= ZIG_ZAG_STEP_SIZE * TRACKING_SPEED_MULTIPLIER
                
                # Check for new detections
                try:
                    distance = self.lidar_data_queue.get_nowait()
                    point = {
                        'distance': distance,
                        'azimuth': self.current_azimuth,
                        'elevation': self.current_elevation,
                        'timestamp': time.time()
                    }
                    yield point
                except Exception:
                    pass
                
            time.sleep(0.01)  # Small delay to prevent CPU overload
