import RPi.GPIO as GPIO
from ..config.settings import STEPPER_PINS, STEPPER_PULSE_DELAY

class StepperController:
    """Controls the stepper motor using GPIO pins."""
    def __init__(self):
        self.setup_gpio()
        self.steps_taken = 0
        self.direction_cw = True
        self.current_angle = 0

    def setup_gpio(self):
        """Configures the RPi's GPIO pins for controlling the DRV8825 stepper driver."""
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in STEPPER_PINS.values():
            GPIO.setup(pin, GPIO.OUT)
        GPIO.output(STEPPER_PINS['ENABLE'], GPIO.LOW)
        GPIO.output(STEPPER_PINS['RESET'], GPIO.HIGH)
        GPIO.output(STEPPER_PINS['M0'], GPIO.HIGH)
        GPIO.output(STEPPER_PINS['M1'], GPIO.HIGH)
        GPIO.output(STEPPER_PINS['M2'], GPIO.HIGH)

    def step(self):
        """Performs a single step."""
        GPIO.output(STEPPER_PINS['STEP'], GPIO.HIGH)
        time.sleep(STEPPER_PULSE_DELAY)
        GPIO.output(STEPPER_PINS['STEP'], GPIO.LOW)
        time.sleep(STEPPER_PULSE_DELAY)
        self.steps_taken += 1

    def set_direction(self, clockwise=True):
        """Sets the rotation direction."""
        self.direction_cw = clockwise
        GPIO.output(STEPPER_PINS['DIR'], GPIO.HIGH if clockwise else GPIO.LOW)

    def cleanup(self):
        """Cleans up GPIO resources."""
        GPIO.cleanup()
