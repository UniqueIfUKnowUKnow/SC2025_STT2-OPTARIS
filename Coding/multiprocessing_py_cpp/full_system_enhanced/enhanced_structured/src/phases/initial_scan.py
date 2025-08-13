import time
import queue
from ..config.settings import (
    DETECTION_THRESHOLD_FACTOR, DETECTION_CONFIDENCE_THRESHOLD,
    TOTAL_DETECTIONS_TO_FIND, STEPPER_SWEEP_DEGREES,
    STEPS_PER_REVOLUTION, SERVO_UPDATE_INTERVAL
)

class InitialScanPhase:
    """Handles the initial scanning phase to find targets."""
    def __init__(self, stepper, servo, lidar_queue, average_distance):
        self.stepper = stepper
        self.servo = servo
        self.lidar_queue = lidar_queue
        self.average_distance = average_distance
        self.steps_for_sweep = int((STEPPER_SWEEP_DEGREES / 360.0) * STEPS_PER_REVOLUTION)
        self.detections_found = 0
        self.consecutive_detections = 0
        self.last_servo_update = time.time()

    def run(self):
        """Executes the initial scan phase."""
        print("Starting initial scan phase...")
        
        # Clear the queue before starting
        with self.lidar_queue.mutex:
            self.lidar_queue.queue.clear()

        while self.detections_found < TOTAL_DETECTIONS_TO_FIND:
            self.stepper.step()
            self.update_servo()
            
            if self.process_lidar_data():
                # Brief pause between detections
                if self.detections_found < TOTAL_DETECTIONS_TO_FIND:
                    print("Pausing for 1 second, then resuming scan...")
                    time.sleep(1)
                    self.consecutive_detections = 0

            if self.stepper.steps_taken >= self.steps_for_sweep:
                self.complete_sweep()

        print(f"\nInitial scan complete! Found {self.detections_found} targets.")
        return True

    def process_lidar_data(self):
        """Processes incoming LiDAR data and checks for detections."""
        try:
            distance = self.lidar_queue.get_nowait()
            
            if distance < (self.average_distance * DETECTION_THRESHOLD_FACTOR):
                self.consecutive_detections += 1
            else:
                self.consecutive_detections = 0

            if self.consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                self.record_detection(distance)
                return True
                
        except queue.Empty:
            pass
        return False

    def record_detection(self, distance):
        """Records and reports a new detection."""
        self.detections_found += 1
        center_angle = 180
        half_sweep = STEPPER_SWEEP_DEGREES / 2.0
        angle_offset = (self.stepper.steps_taken / self.steps_for_sweep) * STEPPER_SWEEP_DEGREES
        
        current_stepper_angle = (center_angle - half_sweep) + angle_offset if self.stepper.direction_cw else (center_angle + half_sweep) - angle_offset

        print("\n" + "="*40)
        print(f"TARGET DETECTED! ({self.detections_found}/{TOTAL_DETECTIONS_TO_FIND})")
        print(f"  -> Distance: {distance} cm (Average was {self.average_distance:.2f} cm)")
        print(f"  -> Stepper Angle: {current_stepper_angle:.1f}°")
        print(f"  -> Servo Angle: {self.servo.current_angle}°")
        print("="*40)

    def update_servo(self):
        """Updates servo position for sweeping motion."""
        if time.time() - self.last_servo_update > SERVO_UPDATE_INTERVAL:
            self.last_servo_update = time.time()
            if self.servo.direction_up:
                self.servo.set_angle(self.servo.current_angle + 1)
                if self.servo.current_angle >= SERVO_SWEEP_END:
                    self.servo.direction_up = False
            else:
                self.servo.set_angle(self.servo.current_angle - 1)
                if self.servo.current_angle <= SERVO_SWEEP_START:
                    self.servo.direction_up = True

    def complete_sweep(self):
        """Handles the completion of a sweep cycle."""
        self.stepper.steps_taken = 0
        self.stepper.direction_cw = not self.stepper.direction_cw
        self.stepper.set_direction(self.stepper.direction_cw)
        self.consecutive_detections = 0
