from ..config.settings import (
    SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE,
    SERVO_SWEEP_START, SERVO_SWEEP_END
)

class ServoController:
    """Controls the servo motor using PWM."""
    def __init__(self, pi):
        self.pi = pi
        self.current_angle = SERVO_SWEEP_START
        self.direction_up = True
        self.set_angle(self.current_angle)

    def set_angle(self, angle):
        """Sets the servo to a specific angle."""
        if angle < SERVO_SWEEP_START:
            angle = SERVO_SWEEP_START
        elif angle > SERVO_SWEEP_END:
            angle = SERVO_SWEEP_END
            
        pulse_width = SERVO_MIN_PULSE + (angle / 180.0) * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)
        self.pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
        self.current_angle = angle

    def cleanup(self):
        """Cleans up PWM resources."""
        self.pi.set_servo_pulsewidth(SERVO_PIN, 0)
