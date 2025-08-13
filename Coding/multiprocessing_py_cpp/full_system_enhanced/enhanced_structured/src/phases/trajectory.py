import time
import queue
import keyboard
from ..config.settings import (
    DETECTION_THRESHOLD_FACTOR, DETECTION_CONFIDENCE_THRESHOLD,
    TRAJECTORY_STEPPER_SWEEP, TRAJECTORY_SERVO_STEP,
    UPDATE_THRESHOLD, STEPS_PER_REVOLUTION,
    SERVO_UPDATE_INTERVAL
)

class TrajectoryPhase:
    """Handles the trajectory following and updating phase."""
    def __init__(self, stepper, servo, lidar_queue, initial_points):
        self.stepper = stepper
        self.servo = servo
        self.lidar_queue = lidar_queue
        self.current_trajectory = initial_points.copy()
        self.tracking_steps = int((TRAJECTORY_STEPPER_SWEEP / 360.0) * STEPS_PER_REVOLUTION)
        self.consecutive_detections = 0
        self.last_servo_update = time.time()
        self.last_detection_time = time.time()
        self.tracking_active = True

    def run(self):
        """Executes the trajectory following phase."""
        print("\nStarting Trajectory Following phase...")
        
        try:
            while self.tracking_active:
                self.stepper.step()
                self.update_servo()
                self.process_lidar_data()
                
                if self.stepper.steps_taken >= self.tracking_steps:
                    self.complete_sweep()
                
                self.check_tracking_status()
                
        except Exception as e:
            print(f"\nError during trajectory tracking: {str(e)}")
        
        self.report_results()
        return self.current_trajectory

    def process_lidar_data(self):
        """Processes incoming LiDAR data and updates trajectory."""
        try:
            distance = self.lidar_queue.get_nowait()
            
            if distance < (self.average_distance * DETECTION_THRESHOLD_FACTOR):
                self.consecutive_detections += 1
            else:
                self.consecutive_detections = 0

            if self.consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                self.update_trajectory(distance)
                
        except queue.Empty:
            pass

    def update_trajectory(self, distance):
        """Updates trajectory points if significant changes are detected."""
        center_angle = 180
        half_sweep = TRAJECTORY_STEPPER_SWEEP / 2.0
        angle_offset = (self.stepper.steps_taken / self.tracking_steps) * TRAJECTORY_STEPPER_SWEEP
        current_stepper_angle = (center_angle - half_sweep) + angle_offset if self.stepper.direction_cw else (center_angle + half_sweep) - angle_offset

        # Check for significant changes in existing points
        for i, point in enumerate(self.current_trajectory):
            if (abs(point['stepper'] - current_stepper_angle) < 10 and 
                abs(point['servo'] - self.servo.current_angle) < 10):
                distance_change = abs(point['distance'] - distance)
                if distance_change > UPDATE_THRESHOLD:
                    self.update_point(i, current_stepper_angle, distance)
                break

    def update_point(self, index, stepper_angle, distance):
        """Updates a specific point in the trajectory."""
        old_point = self.current_trajectory[index]
        self.current_trajectory[index] = {
            'stepper': stepper_angle,
            'servo': self.servo.current_angle,
            'distance': distance
        }
        print("\nTrajectory point updated!")
        print(f"  -> Old distance: {old_point['distance']} cm")
        print(f"  -> New distance: {distance} cm")
        print(f"  -> Change: {abs(old_point['distance'] - distance):.1f} cm")
        
        self.last_detection_time = time.time()
        self.consecutive_detections = 0
        
        # Send updated point to UI (placeholder)
        print(f"Sending updated trajectory point to UI:")
        print(f"  -> Stepper: {stepper_angle:.1f}°")
        print(f"  -> Servo: {self.servo.current_angle}°")
        print(f"  -> Distance: {distance} cm")

    def update_servo(self):
        """Updates servo position with faster movement."""
        if time.time() - self.last_servo_update > (SERVO_UPDATE_INTERVAL / 3):
            self.last_servo_update = time.time()
            if self.servo.direction_up:
                self.servo.set_angle(self.servo.current_angle + TRAJECTORY_SERVO_STEP)
                if self.servo.current_angle >= SERVO_SWEEP_END:
                    self.servo.direction_up = False
            else:
                self.servo.set_angle(self.servo.current_angle - TRAJECTORY_SERVO_STEP)
                if self.servo.current_angle <= SERVO_SWEEP_START:
                    self.servo.direction_up = True

    def complete_sweep(self):
        """Handles the completion of a sweep cycle."""
        self.stepper.steps_taken = 0
        self.stepper.direction_cw = not self.stepper.direction_cw
        self.stepper.set_direction(self.stepper.direction_cw)

    def check_tracking_status(self):
        """Checks if tracking should continue."""
        if time.time() - self.last_detection_time > 10:
            self.tracking_active = False
            print("\nTracking timeout - No significant changes detected.")
        
        if keyboard.is_pressed('q'):
            self.tracking_active = False
            print("\nTracking stopped by user.")

    def report_results(self):
        """Reports the final results of trajectory tracking."""
        print("\nTrajectory following complete.")
        print("\nFinal trajectory points:")
        for i, point in enumerate(self.current_trajectory, 1):
            print(f"Point {i}:")
            print(f"  -> Stepper Angle: {point['stepper']:.1f}°")
            print(f"  -> Servo Angle: {point['servo']}°")
            print(f"  -> Distance: {point['distance']} cm")
