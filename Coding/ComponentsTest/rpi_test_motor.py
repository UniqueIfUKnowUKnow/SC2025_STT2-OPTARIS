# =================================================================
# Python Simple Stepper Motor 360° Test for Raspberry Pi
#
# Purpose: This script is a direct Python conversion of the Arduino
# test code. It performs a basic hardware test to ensure the
# stepper motor can complete a precise 360-degree rotation when
# controlled by a Raspberry Pi.
# =================================================================

import RPi.GPIO as GPIO
import time

# --- Stepper Motor Pins (using BCM numbering) ---
# Updated pin assignments based on your request.
DIR_PIN    = 26
STEP_PIN   = 13
ENABLE_PIN = 23
RESET_PIN  = 19

# --- Microstepping Pins ---
M0_PIN = 0 
M1_PIN = 5
M2_PIN = 6

# --- Motor & Movement Settings ---
# This is the MOST IMPORTANT value to calibrate from your Arduino test.
# Use the value that gave you a perfect 360° rotation.
STEPS_PER_REVOLUTION = 6400

# --- Speed Control ---
# The delay between each step pulse, in seconds.
# A SMALLER value means a FASTER motor speed.
# 0.0002 seconds = 200 microseconds, matching the Arduino code.
PULSE_DELAY_SECONDS = 0.0002

def setup_gpio():
    """Sets up the GPIO pins on the Raspberry Pi."""
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM) # Use Broadcom pin-numbering scheme

    # --- Configure Stepper Motor Pins ---
    GPIO.setup(DIR_PIN, GPIO.OUT)
    GPIO.setup(STEP_PIN, GPIO.OUT)
    GPIO.setup(ENABLE_PIN, GPIO.OUT)
    GPIO.setup(RESET_PIN, GPIO.OUT)

    # --- Configure Microstepping Pins ---
    GPIO.setup(M0_PIN, GPIO.OUT)
    GPIO.setup(M1_PIN, GPIO.OUT)
    GPIO.setup(M2_PIN, GPIO.OUT)

    # --- Set Initial Pin States ---
    GPIO.output(ENABLE_PIN, GPIO.LOW)  # Enable the DRV8825 driver
    GPIO.output(RESET_PIN, GPIO.HIGH)  # Keep the driver out of reset mode

    # Set microstepping to 1/32 for DRV8825 (M0=HIGH, M1=HIGH, M2=HIGH)
    GPIO.output(M0_PIN, GPIO.HIGH)
    GPIO.output(M1_PIN, GPIO.HIGH)
    GPIO.output(M2_PIN, GPIO.HIGH)

def rotate(steps, clockwise):
    """
    Rotates the stepper motor by a specific number of steps.
    :param steps: The number of steps to rotate.
    :param clockwise: The direction of rotation (True for CW, False for CCW).
    """
    # Set the direction of rotation
    GPIO.output(DIR_PIN, GPIO.HIGH if clockwise else GPIO.LOW)

    # Pulse the STEP pin for the specified number of steps
    for _ in range(steps):
        GPIO.output(STEP_PIN, GPIO.HIGH)
        time.sleep(PULSE_DELAY_SECONDS)
        GPIO.output(STEP_PIN, GPIO.LOW)
        time.sleep(PULSE_DELAY_SECONDS)

def main():
    """Main function to run the test loop."""
    print("Starting stepper motor test. Press Ctrl+C to exit.")
    setup_gpio()
    try:
        while True:
            # --- Rotate 360 degrees clockwise ---
            print("Rotating 360 degrees clockwise...")
            rotate(STEPS_PER_REVOLUTION, True)
            time.sleep(2) # Wait for 2 seconds

            # --- Rotate 360 degrees counter-clockwise ---
            print("Rotating 360 degrees counter-clockwise...")
            rotate(STEPS_PER_REVOLUTION, False)
            time.sleep(2) # Wait for 2 seconds

    except KeyboardInterrupt:
        # This block will run when you press Ctrl+C
        print("\nProgram stopped by user.")
    finally:
        # This block will run no matter how the try block exits
        print("Cleaning up GPIO pins.")
        GPIO.cleanup() # Reset GPIO pins to their default state

# =================================================================
# SCRIPT ENTRY POINT
# =================================================================
if __name__ == '__main__':
    main()
