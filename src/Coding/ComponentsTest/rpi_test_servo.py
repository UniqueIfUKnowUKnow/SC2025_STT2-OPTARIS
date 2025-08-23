# =================================================================
# Python Simple Servo Motor Sweep Test for Raspberry Pi
#
# Purpose: This script tests the functionality of a servo motor
# (like the MG9S) by sweeping it back and forth through its
# full range of motion.
#
# This version uses the 'pigpio' library for a more stable,
# jitter-free PWM signal compared to RPi.GPIO.
# =================================================================

import pigpio
import time

# --- Servo Pin (using BCM numbering) ---
SERVO_PIN = 2

# --- Servo Pulse Width Settings (in microseconds) ---
# These values are typical for MG90S servos. You can fine-tune them
# if the servo doesn't reach the full 0 or 180-degree position.
MIN_PULSE_WIDTH = 500  # Corresponds to 0 degrees
MAX_PULSE_WIDTH = 2500 # Corresponds to 180 degrees

def set_angle(pi, angle):
    """
    Sets the servo to a specific angle using precise pulse widths.
    :param pi: The pigpio connection object.
    :param angle: The desired angle (0-180).
    """
    # Calculate the required pulse width for the given angle
    pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
    # A small delay for the servo to physically move
    time.sleep(0.01)

def main():
    """Main function to run the servo test loop."""
    print("Starting servo sweep test using pigpio. Press Ctrl+C to exit.")

    # Connect to the pigpio daemon
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Is it running?")
        return

    try:
        while True:
            # --- Sweep from 0 to 180 degrees ---
            print("Sweeping from 0 to 180 degrees...")
            for angle in range(181): # Loop from 0 to 180 inclusive
                set_angle(pi, angle)
            
            time.sleep(1) # Pause at the end

            # --- Sweep from 180 back to 0 degrees ---
            print("Sweeping from 180 to 0 degrees...")
            for angle in range(180, -1, -1): # Loop from 180 down to 0 inclusive
                set_angle(pi, angle)

            time.sleep(1) # Pause at the end

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        # This block will run no matter how the try block exits
        print("Stopping servo and cleaning up.")
        pi.set_servo_pulsewidth(SERVO_PIN, 0) # Turn off the servo signal
        pi.stop() # Disconnect from the pigpio daemon

# =================================================================
# SCRIPT ENTRY POINT
# =================================================================
if __name__ == '__main__':
    main()
