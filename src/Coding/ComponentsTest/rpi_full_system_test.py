# =================================================================
# Python Full System Test for Raspberry Pi
#
# Purpose: This script tests the simultaneous operation of a stepper
# motor, a servo motor, and a TFmini-S LiDAR sensor.
#
# It uses multithreading to read from the LiDAR sensor in the
# background without interrupting the motor movements.
# =================================================================

import RPi.GPIO as GPIO
import pigpio
import time
import serial
import threading
import queue

# --- Pin & Port Configuration (BCM numbering) ---
# Stepper Motor Pins
DIR_PIN    = 26
STEP_PIN   = 13
ENABLE_PIN = 23
RESET_PIN  = 19
M0_PIN     = 0
M1_PIN     = 5
M2_PIN     = 6

# Servo Motor Pin
SERVO_PIN  = 2

# LiDAR Serial Port
LIDAR_SERIAL_PORT = '/dev/ttyS0'
LIDAR_BAUD_RATE = 115200

# --- Motor & Movement Settings ---
# Stepper Settings
STEPS_PER_REVOLUTION = 6400
STEPPER_PULSE_DELAY = 0.0001 # 100 microseconds

# Servo Settings
MIN_PULSE_WIDTH = 500  # For 0 degrees
MAX_PULSE_WIDTH = 2500 # For 180 degrees
SERVO_UPDATE_INTERVAL = 0.01 # Update servo every 10ms

# --- LiDAR Reader Thread ---
class LidarReader(threading.Thread):
    """A thread dedicated to continuously reading from the TFmini-S LiDAR."""
    def __init__(self, port, baudrate, data_queue):
        super().__init__(daemon=True) # daemon=True allows main to exit cleanly
        self.port = port
        self.baudrate = baudrate
        self.data_queue = data_queue
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        self.frame_header = 0x59
        print("LiDAR Reader thread initialized.")

    def run(self):
        """The main loop for the LiDAR reading thread."""
        while True:
            # Look for the double header 0x59 0x59
            if self.ser.read(1) == b'\x59':
                if self.ser.read(1) == b'\x59':
                    # Read the rest of the 9-byte frame
                    frame = b'\x59\x59' + self.ser.read(7)
                    if len(frame) == 9:
                        # Verify checksum
                        checksum = sum(frame[:-1]) & 0xFF
                        if checksum == frame[8]:
                            distance_cm = frame[2] + (frame[3] << 8)
                            strength = frame[4] + (frame[5] << 8)
                            # Put the valid data into the queue for the main thread
                            self.data_queue.put({'distance_cm': distance_cm, 'strength': strength})

# --- Setup and Control Functions ---
def setup_stepper_gpio():
    """Sets up the GPIO pins for the stepper motor driver."""
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pins = [DIR_PIN, STEP_PIN, ENABLE_PIN, RESET_PIN, M0_PIN, M1_PIN, M2_PIN]
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)
    GPIO.output(ENABLE_PIN, GPIO.LOW)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    GPIO.output(M0_PIN, GPIO.HIGH)
    GPIO.output(M1_PIN, GPIO.HIGH)
    GPIO.output(M2_PIN, GPIO.HIGH)

def set_servo_angle(pi, angle):
    """Calculates and sets the servo to a specific angle."""
    pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

# --- Main Application ---
def main():
    """Main function to run the combined motor and sensor loop."""
    print("Starting full system test. Press Ctrl+C to exit.")

    # --- Setup ---
    setup_stepper_gpio()
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Is it running?")
        return

    # Create a queue to share data between threads
    lidar_data_queue = queue.Queue()
    # Create and start the LiDAR reader thread
    lidar_thread = LidarReader(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, lidar_data_queue)
    lidar_thread.start()

    # --- State Variables for Movement ---
    stepper_steps_taken = 0
    stepper_direction_cw = True
    GPIO.output(DIR_PIN, GPIO.HIGH)
    servo_angle = 0
    servo_direction_up = True
    last_servo_update = time.time()

    try:
        while True:
            # --- Motor Control Logic (as before) ---
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
            stepper_steps_taken += 1

            if stepper_steps_taken >= STEPS_PER_REVOLUTION:
                stepper_steps_taken = 0
                stepper_direction_cw = not stepper_direction_cw
                GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)
                print(f"Stepper reversing direction. Now moving {'CW' if stepper_direction_cw else 'CCW'}.")
                time.sleep(1)

            if time.time() - last_servo_update > SERVO_UPDATE_INTERVAL:
                last_servo_update = time.time()
                if servo_direction_up:
                    servo_angle += 1
                    if servo_angle >= 180: servo_direction_up = False
                else:
                    servo_angle -= 1
                    if servo_angle <= 0: servo_direction_up = True
                set_servo_angle(pi, servo_angle)

            # --- LiDAR Data Processing Logic ---
            # Check if the LiDAR thread has put any new data in the queue
            try:
                lidar_data = lidar_data_queue.get_nowait()
                # If we get data, print it along with the current motor positions
                current_stepper_angle = (stepper_steps_taken / STEPS_PER_REVOLUTION) * 360.0
                print(
                    f"LiDAR Dist: {lidar_data['distance_cm']} cm | "
                    f"Strength: {lidar_data['strength']} | "
                    f"Stepper Angle: {current_stepper_angle:.1f}° | "
                    f"Servo Angle: {servo_angle}°"
                )
            except queue.Empty:
                # This is normal, it just means no new LiDAR data has arrived yet
                pass

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Cleaning up...")
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        GPIO.cleanup()

# =================================================================
# SCRIPT ENTRY POINT
# =================================================================
if __name__ == '__main__':
    main()
