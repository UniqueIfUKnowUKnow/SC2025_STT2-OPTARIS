import time
import math
import numpy as np
from constants import *
from move_motors import set_servo_angle, stepper_step
import RPi.GPIO as GPIO

class AssuranceScan:
    def __init__(self, pi, lidar_data_queue):
        self.pi = pi
        self.lidar_data_queue = lidar_data_queue
        self.detected_points = []
        self.current_azimuth = 0
        self.current_elevation = SERVO_SWEEP_START
        self.scan_direction = 1  # 1 for right, -1 for left
        self.elevation_direction = 1  # 1 for up, -1 for down

    def perform_zigzag_scan(self, avg_background_distance):
        """
        Performs zigzag scanning pattern to find additional points after initial detection.
        Returns coordinates when a point is detected.
        """
        while len(self.detected_points) < POINTS_NEEDED:
            # Move in zigzag pattern
            if self.current_azimuth >= STEPPER_SWEEP_DEGREES:
                self.scan_direction = -1  # Start moving left
                # Increment elevation
                self.current_elevation += ZIG_ZAG_ELEVATION_INCREMENT * self.elevation_direction
                if self.current_elevation > SERVO_SWEEP_END:
                    self.elevation_direction = -1
                    self.current_elevation = SERVO_SWEEP_END
                elif self.current_elevation < SERVO_SWEEP_START:
                    self.elevation_direction = 1
                    self.current_elevation = SERVO_SWEEP_START
                set_servo_angle(self.pi, self.current_elevation)
            elif self.current_azimuth <= 0:
                self.scan_direction = 1  # Start moving right

            # Move stepper motor
            GPIO.output(DIR_PIN, GPIO.HIGH if self.scan_direction == 1 else GPIO.LOW)
            steps_to_move = int((ZIG_ZAG_STEP_SIZE / 360.0) * STEPS_PER_REVOLUTION)
            for _ in range(steps_to_move):
                stepper_step()
                self.current_azimuth += ZIG_ZAG_STEP_SIZE * self.scan_direction

            # Check for detection
            try:
                distance = self.lidar_data_queue.get_nowait()
                if abs(distance - avg_background_distance) > (avg_background_distance * DETECTION_THRESHOLD_FACTOR):
                    point = {
                        'distance': distance,
                        'azimuth': self.current_azimuth,
                        'elevation': self.current_elevation,
                        'timestamp': time.time()
                    }
                    self.detected_points.append(point)
                    yield point

            except Exception:
                pass

            time.sleep(0.01)  # Small delay to prevent CPU overload

    def get_detected_points(self):
        """Returns all currently detected points."""
        return self.detected_points
