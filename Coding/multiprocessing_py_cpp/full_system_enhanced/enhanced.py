# -*- coding: utf-8 -*-
# /usr/bin/python3
"""
Combined TFmini-S LiDAR Scanner with Stepper and Servo Control

This script combines the TFminiS class functionality with the scanner application.
It includes:
- TFminiS class for proper sensor management and frequency control
- Background calibration with stepper and servo sweep
- Continuous scanning loop with re-detection capability
- Configurable refresh rate for the LiDAR sensor

Hardware Connections:
- Raspberry Pi Pin 4 (5V)       -> TFmini-S Red Wire (+5V)
- Raspberry Pi Pin 6 (Ground)   -> TFmini-S Black Wire (GND)
- Raspberry Pi Pin 8 (GPIO 14, TXD) -> TFmini-S White Wire (RXD)
- Raspberry Pi Pin 10 (GPIO 15, RXD) -> TFmini-S Green Wire (TXD)

Stepper Motor (DRV8825) and Servo connections as per original configuration.
"""

# --- Standard Library Imports ---
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import pigpio            # For stable, hardware-based PWM for the servo
import time              # For creating delays
import serial            # For reading data from the LiDAR's serial port
import threading         # To run the LiDAR reader in the background
import queue             # For thread-safe data sharing between threads
import statistics        # For easily calculating the average distance

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

# LiDAR Serial Port Configuration
LIDAR_SERIAL_PORT = '/dev/ttyS0'
LIDAR_BAUD_RATE = 115200
# Define the desired refresh rate in Hz (e.g., 1, 10, 50, 100, 250).
# The TFmini-S supports a range of refresh rates from 1Hz up to 1000Hz.
# The maximum refresh rate depends on the baud rate. At 115200, the maximum is 250Hz.
REFRESH_RATE_HZ = 100

# --- Motor & Movement Settings ---
STEPS_PER_REVOLUTION = 6400
STEPPER_PULSE_DELAY = 0.0001
MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500
SERVO_UPDATE_INTERVAL = 0.01

# --- Stepper and Servo Sweep Settings ---
STEPPER_SWEEP_DEGREES = 40
STEPS_FOR_SWEEP = int((STEPPER_SWEEP_DEGREES / 360.0) * STEPS_PER_REVOLUTION)
SERVO_SWEEP_START = 5
SERVO_SWEEP_END = 25

# --- Calibration & Detection Settings ---
CALIBRATION_SWEEPS = 2
DETECTION_THRESHOLD_FACTOR = 0.8
DETECTION_CONFIDENCE_THRESHOLD = 5
TOTAL_DETECTIONS_TO_FIND = 3


class TFminiS:
    """
    A class to manage communication with the TFmini-S LiDAR sensor.
    """
    def __init__(self, port, baudrate):
        """
        Initializes the sensor object and opens the serial port.
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        # The standard data frame from the sensor is 9 bytes long.
        self.frame_length = 9
        # The header bytes that signify the start of a data frame.
        self.frame_header = 0x59

    def open_serial(self):
        """
        Tries to open the serial port. Returns True on success, False on failure.
        """
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            # Flush any old data in the buffer
            self.ser.flushInput()
            if self.ser.isOpen():
                print(f"Successfully opened serial port: {self.port} at {self.baudrate} baud.")
                return True
            else:
                print(f"Failed to open serial port: {self.port}.")
                return False
        except serial.SerialException as e:
            print(f"Error opening serial port {self.port}: {e}")
            return False

    def close_serial(self):
        """
        Closes the serial port if it's open.
        """
        if self.ser and self.ser.isOpen():
            self.ser.close()
            print("Serial port closed.")
    
    def set_refresh_rate(self, rate_hz):
        """
        Sends a command to the sensor to set the refresh rate in Hz.
        The rate should be between 1 and 1000.
        """
        if not self.ser or not self.ser.isOpen():
            print("Serial port is not open. Cannot set refresh rate.")
            return False
            
        if not 1 <= rate_hz <= 1000:
            print(f"Invalid refresh rate: {rate_hz}. Rate must be between 1 and 1000 Hz.")
            return False
            
        rate_low_byte = rate_hz & 0xFF
        rate_high_byte = (rate_hz >> 8) & 0xFF
        
        command = [0x42, 0x57, 0x02, 0x00, rate_low_byte, rate_high_byte, 0x00, 0x00]
        
        # Calculate the checksum: sum of bytes from 0x02 to 0x00
        checksum = (sum(command[2:]) & 0xFF)
        command.append(checksum)
        
        # Convert the list of integers to a byte string
        command_bytes = bytes(command)
        
        self.ser.write(command_bytes)
        
        # After sending the command, the sensor responds. We can read and verify.
        # A simple flush and a short delay should be enough to let the sensor process the command.
        self.ser.flushInput()
        time.sleep(0.1) # A small delay to ensure the sensor has time to process the command.
        
        print(f"Attempted to set refresh rate to {rate_hz} Hz.")
        return True

    def read_data(self):
        """
        Reads a 9-byte data frame from the sensor, validates it, and parses it.
        Returns a tuple (distance, strength, temperature) or (None, None, None) if data is invalid.
        """
        while True:
            # Check if there's enough data in the buffer to start reading.
            if self.ser.in_waiting >= self.frame_length:
                # Read one byte to look for the first header byte.
                byte1 = self.ser.read(1)
                if not byte1:
                    continue # Timeout, loop again

                # Check if the first byte is the header.
                if ord(byte1) == self.frame_header:
                    # Read the next byte to check for the second header byte.
                    byte2 = self.ser.read(1)
                    if not byte2:
                        continue # Timeout

                    if ord(byte2) == self.frame_header:
                        # If we have two header bytes, read the remaining 7 bytes of the frame.
                        remaining_bytes = self.ser.read(self.frame_length - 2)

                        # Construct the full frame.
                        frame = byte1 + byte2 + remaining_bytes

                        # Verify checksum to ensure data integrity.
                        checksum = frame[-1]
                        calculated_checksum = sum(frame[:-1]) & 0xFF # Lower 8 bits of the sum

                        if checksum == calculated_checksum:
                            # If checksum is valid, parse the data.
                            distance = frame[2] + (frame[3] << 8)
                            strength = frame[4] + (frame[5] << 8)
                            # Temperature calculation according to the datasheet.
                            temperature_raw = frame[6] + (frame[7] << 8)
                            temperature = (temperature_raw / 8.0) - 256.0

                            return distance, strength, temperature
                        else:
                            # Checksum failed, data is corrupt.
                            # We can flush the input to try and re-sync.
                            self.ser.flushInput()
                            print("Checksum error. Flushing buffer and retrying.")
                            return None, None, None
            else:
                # Not enough data in the buffer, wait for more data to arrive.
                # This is a non-busy wait as read() with a timeout would have already waited.
                time.sleep(0.001) # A very short sleep to prevent a tight loop from consuming 100% CPU


# --- LiDAR Reader Thread ---
class LidarReader(threading.Thread):
    """A dedicated thread that continuously reads data from the TFmini-S LiDAR using the TFminiS class."""
    def __init__(self, tfmini_sensor, data_queue):
        super().__init__(daemon=True)
        self.tfmini_sensor = tfmini_sensor
        self.data_queue = data_queue
        print("LiDAR Reader thread initialized with TFminiS class.")

    def run(self):
        while True:
            try:
                distance_cm, strength, temperature = self.tfmini_sensor.read_data()
                if distance_cm is not None and distance_cm > 0 and distance_cm != 65535:
                    self.data_queue.put(distance_cm)
            except Exception as e:
                print(f"Error in LiDAR reader thread: {e}")
                time.sleep(0.1)


# --- Setup and Control Functions ---
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

def set_servo_angle(pi, angle):
    """Calculates the required pulse width and sets the servo to a specific angle."""
    pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

# --- Main Application ---
def main():
    """
    The main entry point of the program. It handles setup, state management,
    and the main control loop for the calibration and scanning process.
    """
    print("--- TFmini-S LiDAR Scanner with Stepper and Servo Control ---")
    
    # --- Setup ---
    setup_stepper_gpio()
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Is it running?")
        return

    # Initialize TFminiS sensor
    lidar_sensor = TFminiS(port=LIDAR_SERIAL_PORT, baudrate=LIDAR_BAUD_RATE)
    if not lidar_sensor.open_serial():
        print("Exiting program. Please check your connections and serial port configuration.")
        GPIO.cleanup()
        pi.stop()
        return

    # Set the refresh rate
    lidar_sensor.set_refresh_rate(REFRESH_RATE_HZ)

    # Initialize LiDAR reader thread with the TFminiS sensor
    lidar_data_queue = queue.Queue()
    lidar_thread = LidarReader(lidar_sensor, lidar_data_queue)
    lidar_thread.start()

    # --- State Machine and Variables ---
    states = ["CALIBRATING", "SCANNING", "FINISHED"]
    current_state = states[0]
    print(f"Current State: {current_state}")

    stepper_steps_taken = 0
    stepper_direction_cw = True
    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    # Initialize servo for sweeping motion
    servo_angle = SERVO_SWEEP_START
    servo_direction_up = True
    set_servo_angle(pi, servo_angle)
    last_servo_update = time.time()
    print(f"Servo sweep range set to {SERVO_SWEEP_START}° - {SERVO_SWEEP_END}°.")

    calibration_distances = []
    sweeps_completed = 0
    average_distance = 0
    
    # Counter for the detection loop.
    detections_found = 0
    consecutive_detections = 0

    try:
        # =================================================================
        # 1. CALIBRATION PHASE
        # =================================================================
        print("\nStarting calibration phase...")
        while current_state == "CALIBRATING":
            # --- Interleaved Motor Control ---
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
            stepper_steps_taken += 1

            # --- Servo sweep logic ---
            if time.time() - last_servo_update > SERVO_UPDATE_INTERVAL:
                last_servo_update = time.time()
                if servo_direction_up:
                    servo_angle += 1
                    if servo_angle >= SERVO_SWEEP_END: servo_direction_up = False
                else:
                    servo_angle -= 1
                    if servo_angle <= SERVO_SWEEP_START: servo_direction_up = True
                set_servo_angle(pi, servo_angle)

            try:
                distance = lidar_data_queue.get_nowait()
                calibration_distances.append(distance)
            except queue.Empty:
                pass

            if stepper_steps_taken >= STEPS_FOR_SWEEP:
                stepper_steps_taken = 0
                stepper_direction_cw = not stepper_direction_cw
                GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)
                sweeps_completed += 0.5
                print(f"Calibration sweep {int(sweeps_completed * 2)} of {CALIBRATION_SWEEPS * 2} halfs completed...")

                if sweeps_completed >= CALIBRATION_SWEEPS:
                    if calibration_distances:
                        average_distance = statistics.mean(calibration_distances)
                        print("\n" + "="*30)
                        print("CALIBRATION COMPLETE")
                        print(f"Average Background Distance: {average_distance:.2f} cm")
                        print("="*30 + "\n")
                        current_state = states[1]
                        print(f"Current State: {current_state}")
                    else:
                        print("Calibration Failed: No LiDAR data collected.")
                        current_state = "FINISHED"
        
        # Reset stepper state for the next phase
        stepper_steps_taken = 0
        
        # Clear the LiDAR data queue before starting the scan.
        with lidar_data_queue.mutex:
            lidar_data_queue.queue.clear()
        
        # =================================================================
        # 2. INITIAL SCAN & RE-DETECTION PHASE
        # =================================================================
        print("Starting detection phase...")
        # The main loop now continues until 3 detections are found.
        while detections_found < TOTAL_DETECTIONS_TO_FIND:
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
            stepper_steps_taken += 1

            # --- Servo sweep logic ---
            if time.time() - last_servo_update > SERVO_UPDATE_INTERVAL:
                last_servo_update = time.time()
                if servo_direction_up:
                    servo_angle += 1
                    if servo_angle >= SERVO_SWEEP_END: servo_direction_up = False
                else:
                    servo_angle -= 1
                    if servo_angle <= SERVO_SWEEP_START: servo_direction_up = True
                set_servo_angle(pi, servo_angle)

            # --- Detection Logic ---
            try:
                distance = lidar_data_queue.get_nowait()
                
                if distance < (average_distance * DETECTION_THRESHOLD_FACTOR):
                    consecutive_detections += 1
                else:
                    consecutive_detections = 0

                if consecutive_detections >= DETECTION_CONFIDENCE_THRESHOLD:
                    center_angle = 180
                    half_sweep = STEPPER_SWEEP_DEGREES / 2.0
                    angle_offset = (stepper_steps_taken / STEPS_FOR_SWEEP) * STEPPER_SWEEP_DEGREES
                    
                    if stepper_direction_cw:
                        current_stepper_angle = (center_angle - half_sweep) + angle_offset
                    else:
                        current_stepper_angle = (center_angle + half_sweep) - angle_offset

                    detections_found += 1
                    print("\n" + "="*40)
                    print(f"TARGET DETECTED! ({detections_found}/{TOTAL_DETECTIONS_TO_FIND})")
                    print(f"  -> Distance: {distance} cm (Average was {average_distance:.2f} cm)")
                    print(f"  -> Stepper Angle: {current_stepper_angle:.1f}°")
                    print(f"  -> Servo Angle: {servo_angle}°")
                    print("="*40)
                    
                    # Pause and prepare for the next scan.
                    if detections_found < TOTAL_DETECTIONS_TO_FIND:
                        print("Pausing for 1 second, then resuming scan...")
                        time.sleep(1)
                        # Clear the queue and confidence to avoid instant re-detection
                        with lidar_data_queue.mutex:
                            lidar_data_queue.queue.clear()
                        consecutive_detections = 0
                    
            except queue.Empty:
                pass

            # Reverse direction at the end of the sweep
            if stepper_steps_taken >= STEPS_FOR_SWEEP:
                stepper_steps_taken = 0
                stepper_direction_cw = not stepper_direction_cw
                GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW)
                consecutive_detections = 0

        print(f"\nAll {TOTAL_DETECTIONS_TO_FIND} targets detected. Scanning complete.")

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        print("Cleaning up...")
        # Stop servo
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        # Close LiDAR sensor
        lidar_sensor.close_serial()
        # Clean up GPIO
        GPIO.cleanup()
        print("Program terminated.")

if __name__ == '__main__':
    main()