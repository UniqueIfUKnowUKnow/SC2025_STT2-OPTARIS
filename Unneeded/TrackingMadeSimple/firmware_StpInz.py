# firmware_StpInz.py
# Contains all the code for initializing data input/output to different modules,
# GPIO setup, stepper motor setup with driver board, servo motor setup, and LiDAR sensor setup

# --- Standard Library Imports ---
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import pigpio            # For stable, hardware-based PWM for the servo
import time              # For creating delays
import serial            # For reading data from the LiDAR's serial port
import threading         # To run the LiDAR reader in the background
import queue             # For thread-safe data sharing between threads
import numpy as np

# --- Constants ---
# Pin & Port Configuration (BCM numbering)
DIR_PIN    = 26
STEP_PIN   = 13
ENABLE_PIN = 23
RESET_PIN  = 19
M0_PIN     = 0
M1_PIN     = 5
M2_PIN     = 6

SERVO_PIN  = 2

LIDAR_SERIAL_PORT = '/dev/ttyS0'
LIDAR_BAUD_RATE = 115200

# Motor & Movement Settings
STEPS_PER_REVOLUTION = 6400
STEPPER_PULSE_DELAY = 0.0001
MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500
SERVO_UPDATE_INTERVAL = 0.01

# Stepper and Servo Sweep Settings
STEPPER_SWEEP_DEGREES = 360
STEPS_FOR_SWEEP = int((STEPPER_SWEEP_DEGREES / 360.0) * STEPS_PER_REVOLUTION)
SERVO_SWEEP_START = 0
SERVO_SWEEP_END = 20

# LiDAR Settings
SENSOR_MAX = 1200  # cm

# --- GPIO Setup Functions ---
def setup_stepper_gpio():
    """Configures the RPi's GPIO pins for controlling the DRV8825 stepper driver."""
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
    print("Stepper motor GPIO setup complete")

def setup_servo_gpio():
    """Initialize pigpio for servo control."""
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Is it running?")
        return None
    print("Servo motor setup complete")
    return pi

def setup_lidar_gpio():
    """Initialize LiDAR serial connection."""
    try:
        ser = serial.Serial(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, timeout=0.1)
        ser.flushInput()
        print("LiDAR sensor setup complete")
        return ser
    except Exception as e:
        print(f"Failed to connect to LiDAR: {e}")
        return None

# --- Motor Control Functions ---
def set_servo_angle(pi, angle):
    """Calculates the required pulse width and sets the servo to a specific angle."""
    if pi and pi.connected:
        pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
        pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)
        print(f"Servo moved to angle: {angle}°")

def stepper_step():
    """Single step of the stepper motor."""
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(STEPPER_PULSE_DELAY)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(STEPPER_PULSE_DELAY)

def reset_stepper_pos(stepper_steps_taken):
    """Reset stepper motor position based on steps tracked from beginning."""
    state = GPIO.input(DIR_PIN)
    if state:
        GPIO.output(DIR_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    
    # Add delay after direction change
    time.sleep(0.001)
    
    if stepper_steps_taken > 0:
        for _ in range(stepper_steps_taken):
            stepper_step()
    
    # Reset to known direction state
    GPIO.output(DIR_PIN, GPIO.HIGH)
    time.sleep(0.001)  # Allow direction to settle
    print("Stepper position reset and direction pin set to HIGH")

# --- LiDAR Reader Class ---
class LidarReader(threading.Thread):
    """A dedicated thread that continuously reads data from the TFmini-S LiDAR."""
    def __init__(self, port, baudrate, data_queue):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.data_queue = data_queue
        self.ser = None
        self.frame_header = 0x59
        self._connect()
        print("LiDAR Reader thread initialized.")

    def _connect(self):
        """Establish serial connection with error handling."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.ser.flushInput()
            print("LiDAR connection established.")
        except Exception as e:
            print(f"Failed to connect to LiDAR: {e}")
            self.ser = None

    def run(self):
        while True:
            try:
                if not self.ser or not self.ser.is_open:
                    print("LiDAR disconnected. Attempting to reconnect...")
                    self._connect()
                    if not self.ser:
                        time.sleep(1)  # Wait before retry
                        continue

                # Read first header byte
                data = self.ser.read(1)
                if not data:  # Handle empty read
                    continue
                    
                if data == b'\x59':
                    # Read second header byte
                    data = self.ser.read(1)
                    if not data:
                        continue
                        
                    if data == b'\x59':
                        frame = b'\x59\x59' + self.ser.read(7)
                        if len(frame) == 9:
                            checksum = sum(frame[:-1]) & 0xFF
                            if checksum == frame[8]:
                                distance_cm = frame[2] + (frame[3] << 8)
                                if distance_cm > 0:
                                    self.data_queue.put(distance_cm)
                                    
            except serial.SerialException as e:
                print(f"LiDAR serial error: {e}")
                self.ser = None
                time.sleep(0.05)  # Brief pause before reconnection attempt
            except Exception as e:
                print(f"Unexpected LiDAR error: {e}")
                time.sleep(0.05)

# --- Main Initialization Function ---
def initialize_firmware():
    """Main function to initialize all firmware components."""
    print("=== FIRMWARE INITIALIZATION STARTING ===")
    
    # Setup GPIO for stepper motor
    setup_stepper_gpio()
    
    # Setup servo motor
    pi = setup_servo_gpio()
    if not pi:
        print("ERROR: Failed to initialize servo motor")
        return None, None, None
    
    # Setup LiDAR sensor
    lidar_ser = setup_lidar_gpio()
    if not lidar_ser:
        print("ERROR: Failed to initialize LiDAR sensor")
        return None, None, None
    
    # Initialize LiDAR data queue and reader thread
    lidar_data_queue = queue.Queue()
    lidar_thread = LidarReader(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, lidar_data_queue)
    lidar_thread.start()
    
    # Set initial positions
    GPIO.output(DIR_PIN, GPIO.HIGH)
    servo_angle = SERVO_SWEEP_START
    set_servo_angle(pi, servo_angle)
    
    print("=== FIRMWARE INITIALIZATION COMPLETE ===")
    print("All systems ready for operation")
    
    return pi, lidar_data_queue, lidar_thread

# --- Cleanup Function ---
def cleanup_firmware(pi):
    """Clean up all GPIO and connections."""
    print("Cleaning up firmware...")
    if pi and pi.connected:
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
    GPIO.cleanup()
    print("Firmware cleanup complete")

if __name__ == '__main__':
    # Test initialization
    pi, lidar_queue, lidar_thread = initialize_firmware()
    if pi:
        try:
            print("Firmware test running... Press Ctrl+C to stop")
            time.sleep(5)  # Run for 5 seconds
        except KeyboardInterrupt:
            print("\nStopping firmware test...")
        finally:
            cleanup_firmware(pi)
    else:
        print("Firmware initialization failed")
