import time
import queue
from ..config.settings import (
    DETECTION_THRESHOLD_FACTOR, DETECTION_CONFIDENCE_THRESHOLD,
    TOTAL_DETECTIONS_TO_FIND, ASSURANCE_STEPPER_SWEEP,
    STEPS_PER_REVOLUTION, SERVO_UPDATE_INTERVAL
)

class AssuranceScanPhase:
    """Handles the assurance scanning phase to verify target positions."""
    def __init__(self, stepper, servo, lidar_queue, average_distance):
        self.stepper = stepper
        self.servo = servo
        self.lidar_queue = lidar_queue
        self.average_distance = average_distance
        self.assurance_steps = int((ASSURANCE_STEPPER_SWEEP / 360.0) * STEPS_PER_REVOLUTION)
        self.detections_verified = 0
        self.detection_points = []
        self.consecutive_detections = 0
        self.last_servo_update = time.time()

    def run(self):
        """Executes the assurance scan phase."""
        print("\nStarting Assurance Scan phase...")
        
        # Clear the queue before starting
        with self.lidar_queue.mutex:
            self.lidar_queue.queue.clear()

        while self.detections_verified < TOTAL_DETECTIONS_TO_FIND:
            self.stepper.step()
            self.update_servo()
            self.process_lidar_data()

            if self.stepper.steps_taken >= self.assurance_steps:
                self.complete_sweep()

        self.report_results()
        return self.detection_points

    def process_lidar_data(self):
        """Processes incoming LiDAR data and verifies detections."""
        try:
            distance = self.lidar_queue.get_nowait()
            
            if distance < (self.average_distance * DETECTION_THRESHOLD_FACTOR):
                self.consecutive_detections += 1
            else:
                self.consecutive_detections = 0

            if self.consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                self.verify_detection(distance)
                
        except queue.Empty:
            pass

    def verify_detection(self, distance):
        """Verifies and records a new detection."""
        center_angle = 180
        half_sweep = ASSURANCE_STEPPER_SWEEP / 2.0
        angle_offset = (self.stepper.steps_taken / self.assurance_steps) * ASSURANCE_STEPPER_SWEEP
        current_stepper_angle = (center_angle - half_sweep) + angle_offset if self.stepper.direction_cw else (center_angle + half_sweep) - angle_offset

        # Check if this is a new detection point
        is_new_point = True
        for point in self.detection_points:
            if (abs(point['stepper'] - current_stepper_angle) < 5 and 
                abs(point['servo'] - self.servo.current_angle) < 5):
                is_new_point = False
                break

        if is_new_point:
            self.record_new_point(current_stepper_angle, distance)

    def record_new_point(self, stepper_angle, distance):
        """Records a new verified detection point."""
        self.detection_points.append({
            'stepper': stepper_angle,
            'servo': self.servo.current_angle,
            'distance': distance
        })
        self.detections_verified += 1
        print("\n" + "="*40)
        print(f"Target Verified! ({self.detections_verified}/{TOTAL_DETECTIONS_TO_FIND})")
        print(f"  -> Distance: {distance} cm")
        print(f"  -> Stepper Angle: {stepper_angle:.1f}°")
        print(f"  -> Servo Angle: {self.servo.current_angle}°")
        print("="*40)
        time.sleep(0.5)
        self.consecutive_detections = 0

    def update_servo(self):
        """Updates servo position with faster movement for zig-zag pattern."""
        if time.time() - self.last_servo_update > (SERVO_UPDATE_INTERVAL / 2):
            self.last_servo_update = time.time()
            if self.servo.direction_up:
                self.servo.set_angle(self.servo.current_angle + 2)
                if self.servo.current_angle >= SERVO_SWEEP_END:
                    self.servo.direction_up = False
            else:
                self.servo.set_angle(self.servo.current_angle - 2)
                if self.servo.current_angle <= SERVO_SWEEP_START:
                    self.servo.direction_up = True

    def complete_sweep(self):
        """Handles the completion of a sweep cycle."""
        self.stepper.steps_taken = 0
        self.stepper.direction_cw = not self.stepper.direction_cw
        self.stepper.set_direction(self.stepper.direction_cw)
        self.consecutive_detections = 0

    def report_results(self):
        """Reports the final results of the assurance scan."""
        print("\nAssurance scan complete! All targets verified.")
        print("\nVerified target positions:")
        for i, point in enumerate(self.detection_points, 1):
            print(f"Target {i}:")
            print(f"  -> Stepper Angle: {point['stepper']:.1f}°")
            print(f"  -> Servo Angle: {point['servo']}°")
            print(f"  -> Distance: {point['distance']} cm")
