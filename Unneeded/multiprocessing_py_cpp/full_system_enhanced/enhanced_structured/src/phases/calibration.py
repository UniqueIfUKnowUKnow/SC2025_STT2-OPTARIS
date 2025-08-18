import time
import statistics
from ..config.settings import (
    CALIBRATION_SWEEPS, STEPS_PER_REVOLUTION,
    STEPPER_SWEEP_DEGREES, SERVO_UPDATE_INTERVAL
)

class CalibrationPhase:
    """Handles the calibration phase of the scanning process."""
    def __init__(self, stepper, servo, lidar_queue):
        self.stepper = stepper
        self.servo = servo
        self.lidar_queue = lidar_queue
        self.calibration_distances = []
        self.steps_for_sweep = int((STEPPER_SWEEP_DEGREES / 360.0) * STEPS_PER_REVOLUTION)
        self.sweeps_completed = 0
        self.last_servo_update = time.time()

    def run(self):
        """Executes the calibration phase."""
        print("Starting calibration phase...")

        while self.sweeps_completed < CALIBRATION_SWEEPS:
            # Stepper control
            self.stepper.step()
            
            # Servo sweep
            if time.time() - self.last_servo_update > SERVO_UPDATE_INTERVAL:
                self.update_servo()

            # LiDAR data collection
            try:
                distance = self.lidar_queue.get_nowait()
                self.calibration_distances.append(distance)
            except queue.Empty:
                pass

            # Check sweep completion
            if self.stepper.steps_taken >= self.steps_for_sweep:
                self.complete_sweep()

        # Calculate and return results
        if self.calibration_distances:
            average_distance = statistics.mean(self.calibration_distances)
            print("\n" + "="*30)
            print("CALIBRATION COMPLETE")
            print(f"Average Background Distance: {average_distance:.2f} cm")
            print("="*30 + "\n")
            return average_distance
        else:
            print("Calibration Failed: No LiDAR data collected.")
            return None

    def update_servo(self):
        """Updates servo position for sweeping motion."""
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
        self.sweeps_completed += 0.5
        print(f"Calibration sweep {int(self.sweeps_completed * 2)} of {CALIBRATION_SWEEPS * 2} halfs completed...")
