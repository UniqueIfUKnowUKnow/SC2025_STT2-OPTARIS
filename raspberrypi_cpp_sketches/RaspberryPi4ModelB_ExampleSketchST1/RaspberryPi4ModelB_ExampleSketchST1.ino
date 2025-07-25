# Import necessary libraries
import RPi.GPIO as GPIO  # For controlling GPIO pins (motors)
import serial            # For UART communication with TFmini-S
import time              # For delays

# --- Pin Definitions (Using BCM numbering scheme) ---
# It's crucial to connect your components to these specific GPIO pins on your Raspberry Pi.
# You can find a GPIO pinout diagram for Raspberry Pi 4 online.

# MG90S Servo (Pan/Horizontal control)
# Connect servo signal pin to Raspberry Pi GPIO 18 (PWM capable pin).
SERVO_PIN = 18

# DRV8825 Stepper Motor Driver (Tilt/Vertical control)
# Connect DRV8825 DIR pin to Raspberry Pi GPIO 5.
STEPPER_DIR_PIN = 5
# Connect DRV8825 STEP pin to Raspberry Pi GPIO 6.
STEPPER_STEP_PIN = 6
# Connect DRV8825 ENABLE pin to Raspberry Pi GPIO 7.
# Set LOW to enable the driver, HIGH to disable.
STEPPER_ENABLE_PIN = 7
# Microstepping pins (MS1, MS2, MS3) on DRV8825.
# These determine the microstepping resolution (e.g., 1/16, 1/32).
STEPPER_MS1_PIN = 2
STEPPER_MS2_PIN = 3
STEPPER_MS3_PIN = 4

# TFmini-S LiDAR sensor (UART communication)
# Raspberry Pi's default UART pins are GPIO 14 (TX) and GPIO 15 (RX).
# Connect TFmini-S TX to Raspberry Pi RX (GPIO 15).
# Connect TFmini-S RX to Raspberry Pi TX (GPIO 14).
# Ensure you enable the serial port and disable console over serial in raspi-config.
TFMINI_SERIAL_PORT = "/dev/ttyS0" # Or "/dev/ttyAMA0" on older Pis or if configured differently
TFMINI_BAUDRATE = 115200

# --- Global Objects / Variables ---
# Servo PWM object
pan_servo_pwm = None

# Serial object for TFmini-S
tfmini_serial = None

# --- Stepper Motor Configuration ---
# STEPS_PER_REVOLUTION: Number of full steps for one revolution of your stepper motor.
# Common values are 200 (for 1.8 degree/step) or 400 (for 0.9 degree/step).
STEPS_PER_REVOLUTION = 200
# MICROSTEPS: The microstepping setting for your DRV8825 driver.
# This example assumes 1/16 microstepping, which is common.
# Ensure your MS1, MS2, MS3 pins are configured accordingly on the DRV8825.
MICROSTEPS = 16
# STEPS_PER_DEGREE_TILT: Calculated steps needed to move the stepper by one degree.
# This is a theoretical value. Actual physical gearing will affect this.
STEPS_PER_DEGREE_TILT = (STEPS_PER_REVOLUTION * MICROSTEPS) / 360

# --- TFmini-S Data Variables ---
distance = 0  # Distance reading from TFmini-S in centimeters.
strength = 0  # Signal strength reading from TFmini-S.

# --- Tracking State Variables ---
# Defines the different states of the drone tracking system.
SCANNING = 0  # Actively searching for a drone.
TRACKING = 1  # Drone detected, actively following it.
current_state = SCANNING  # Initial state is SCANNING.

# Drone target position (angles)
# These variables store the last known or desired pan/tilt angles of the drone.
target_pan_angle = 90  # Initial center position for servo (0-180 degrees).
# For tilt, we'll use a conceptual angle, as the stepper's range might vary.
# A value like 90 could represent a horizontal tilt.
target_tilt_angle = 90

# Scan parameters
# Defines the range and step size for the horizontal scanning motion.
SCAN_START_PAN = 45   # Starting pan angle for scanning (degrees).
SCAN_END_PAN = 135    # Ending pan angle for scanning (degrees).
SCAN_STEP_PAN = 5     # Increment/decrement step size for pan during scan (degrees).
SCAN_DELAY_SEC = 0.05 # Delay after each scan step to allow servo to move and sensor to read (seconds).

# Drone detection range (in centimeters)
# The system will consider an object a "drone" if its distance falls within this range.
DETECTION_RANGE_MIN = 800  # Minimum distance for detection (8 meters).
DETECTION_RANGE_MAX = 1200 # Maximum distance for detection (12 meters).
# Minimum signal strength for a valid detection.
MIN_STRENGTH = 100

# --- Helper Functions ---

def send_message(message):
    """Custom function to print messages to the console."""
    print(f"MESSAGE: {message}")

# --- Servo Control ---
def set_servo_angle(angle):
    """
    Sets the servo to a specified angle (0-180 degrees).
    Calculates the PWM duty cycle based on the angle.
    Typical servo: 1ms pulse for 0 deg, 1.5ms for 90 deg, 2ms for 180 deg.
    PWM frequency: 50 Hz (20ms period).
    Duty cycle = (pulse_width / period) * 100
    """
    if not 0 <= angle <= 180:
        send_message(f"Warning: Servo angle {angle} out of 0-180 range.")
        return

    # Map angle (0-180) to duty cycle (2.5-12.5)
    # This range might need fine-tuning for your specific servo.
    # 2.5% duty cycle corresponds to 0.5ms pulse (often 0 degrees)
    # 12.5% duty cycle corresponds to 2.5ms pulse (often 180 degrees)
    # A more common range is 5% (1ms) to 10% (2ms) for 0-180 degrees.
    # Let's use the common 5-10% range for 0-180 degrees.
    duty_cycle = 2.5 + (angle / 180.0) * 10.0 # From 2.5% to 12.5%
    pan_servo_pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.3) # Give servo time to move

# --- Stepper Motor Functions ---

def set_stepper_microstepping(ms):
    """
    Sets the microstepping resolution for the DRV8825 driver.
    ms: Desired microstepping value (1, 2, 4, 8, 16, 32).
    This function configures the MS1, MS2, MS3 pins accordingly.
    """
    # Reset all microstepping pins to LOW initially.
    GPIO.output(STEPPER_MS1_PIN, GPIO.LOW)
    GPIO.output(STEPPER_MS2_PIN, GPIO.LOW)
    GPIO.output(STEPPER_MS3_PIN, GPIO.LOW)

    # Set pins based on the desired microstepping value.
    if ms == 1: # Full step (all MS pins LOW)
        pass
    elif ms == 2: # 1/2 step
        GPIO.output(STEPPER_MS1_PIN, GPIO.HIGH)
    elif ms == 4: # 1/4 step
        GPIO.output(STEPPER_MS2_PIN, GPIO.HIGH)
    elif ms == 8: # 1/8 step
        GPIO.output(STEPPER_MS1_PIN, GPIO.HIGH)
        GPIO.output(STEPPER_MS2_PIN, GPIO.HIGH)
    elif ms == 16: # 1/16 step
        GPIO.output(STEPPER_MS3_PIN, GPIO.HIGH)
    elif ms == 32: # 1/32 step
        GPIO.output(STEPPER_MS1_PIN, GPIO.HIGH)
        GPIO.output(STEPPER_MS3_PIN, GPIO.HIGH)
    else:
        send_message("Invalid microstepping value. Defaulting to 1 (Full Step).")
    time.sleep(0.001) # Small delay to allow pin states to settle.

def move_stepper(steps, direction):
    """
    Moves the stepper motor by a given number of steps in a specified direction.
    steps: The number of microsteps to move.
    direction: 0 for one direction (e.g., clockwise), 1 for the other (e.g., counter-clockwise).
               Depends on your wiring.
    """
    GPIO.output(STEPPER_DIR_PIN, direction) # Set the direction.
    for _ in range(steps):
        GPIO.output(STEPPER_STEP_PIN, GPIO.HIGH) # Pulse the STEP pin HIGH.
        time.sleep(0.0005) # Adjust this delay for desired speed (lower value = faster).
        GPIO.output(STEPPER_STEP_PIN, GPIO.LOW)  # Pulse the STEP pin LOW.
        time.sleep(0.0005) # Adjust this delay for desired speed.

# --- TFmini-S Functions ---

def setup_tfmini_s():
    """Initializes the serial port for TFmini-S communication."""
    global tfmini_serial
    try:
        tfmini_serial = serial.Serial(TFMINI_SERIAL_PORT, TFMINI_BAUDRATE, timeout=1)
        send_message("TFmini-S initialized.")
    except serial.SerialException as e:
        send_message(f"Error opening serial port for TFmini-S: {e}")
        send_message("Please check serial port configuration and permissions.")
        exit() # Exit if serial port cannot be opened.

def read_tfmini_s():
    """
    Reads data from TFmini-S via UART.
    This function parses the 9-byte data frame from the TFmini-S.
    Returns true if a valid frame with correct checksum is received, false otherwise.
    """
    global distance, strength
    frame = bytearray(9)
    i = 0

    if tfmini_serial is None:
        return False

    while tfmini_serial.in_waiting > 0:
        data = tfmini_serial.read(1)[0] # Read one byte

        if i == 0: # First byte should be 0x59 (start of frame).
            if data == 0x59:
                frame[i] = data
                i += 1
            else:
                i = 0 # Reset if not start byte
        elif i == 1: # Second byte should also be 0x59.
            if data == 0x59:
                frame[i] = data
                i += 1
            else:
                i = 0 # Reset if not start byte
        else: # Store subsequent bytes until the frame is complete.
            frame[i] = data
            i += 1
            if i == 9: # Full 9-byte frame received.
                # Extract distance (bytes 2 and 3) and strength (bytes 4 and 5).
                dist = frame[2] + (frame[3] << 8) # Distance LSB + MSB.
                str_val = frame[4] + (frame[5] << 8)  # Strength LSB + MSB.
                checksum = frame[8]               # Checksum byte.

                # Calculate checksum for bytes 0-7.
                calculated_checksum = sum(frame[j] for j in range(8)) % 256 # Modulo 256 for byte sum

                # Check if calculated checksum matches the received checksum.
                if calculated_checksum == checksum:
                    distance = dist    # Update global distance variable.
                    strength = str_val     # Update global strength variable.
                    i = 0              # Reset index for the next frame.
                    return True        # Valid frame received.
                else:
                    # Checksum mismatch, discard frame and reset.
                    send_message("TFmini-S checksum error.")
                    i = 0
    return False # No valid frame received yet.

# --- Drone Tracking Logic ---

def scan_for_drone():
    """Scans the environment by moving the servo and reading the TFmini-S."""
    global current_state, target_pan_angle
    global current_pan, pan_direction # Make these static-like by defining them outside and using `nonlocal` if inside nested func

    # Initialize static-like variables if they don't exist
    if not hasattr(scan_for_drone, 'current_pan'):
        scan_for_drone.current_pan = SCAN_START_PAN
        scan_for_drone.pan_direction = 1

    set_servo_angle(scan_for_drone.current_pan) # Move servo to the current scan position.
    time.sleep(SCAN_DELAY_SEC)       # Wait for the servo to reach its position.

    if read_tfmini_s(): # Attempt to read distance from TFmini-S.
        print(f"Scanning at Pan: {scan_for_drone.current_pan} deg, Distance: {distance} cm, Strength: {strength}")

        # Check if an object is within the detection range and has sufficient signal strength.
        if DETECTION_RANGE_MIN <= distance <= DETECTION_RANGE_MAX and strength >= MIN_STRENGTH:
            send_message("Drone detected!")
            target_pan_angle = scan_for_drone.current_pan # Store the pan angle where the drone was detected.
            # For tilt, we assume the initial tilt is adequate for detection.
            # In a real system, you might also scan vertically with the stepper.
            current_state = TRACKING # Transition to the TRACKING state.
            return # Exit the scan function to start tracking.

    # Move to the next pan position for scanning.
    scan_for_drone.current_pan += (SCAN_STEP_PAN * scan_for_drone.pan_direction)

    # Reverse scanning direction if limits are reached.
    if scan_for_drone.current_pan >= SCAN_END_PAN:
        scan_for_drone.pan_direction = -1
        scan_for_drone.current_pan = SCAN_END_PAN # Ensure it doesn't go beyond the end limit.
    elif scan_for_drone.current_pan <= SCAN_START_PAN:
        scan_for_drone.pan_direction = 1
        scan_for_drone.current_pan = SCAN_START_PAN # Ensure it doesn't go below the start limit.

def track_drone():
    """Tracks the drone once it has been detected."""
    global current_state
    global last_read_time # Make this static-like

    # Initialize static-like variable if it doesn't exist
    if not hasattr(track_drone, 'last_read_time'):
        track_drone.last_read_time = time.time()

    if read_tfmini_s(): # Continuously read distance while tracking.
        track_drone.last_read_time = time.time() # Update last read time.
        print(f"Tracking - Distance: {distance} cm, Strength: {strength}")

        # Check if the drone is still within the detection range and has sufficient strength.
        # If not, it means the drone has moved out of the sensor's narrow beam or is too far/close.
        if distance < DETECTION_RANGE_MIN or distance > DETECTION_RANGE_MAX or strength < MIN_STRENGTH:
            send_message("Drone lost! Re-scanning...")
            current_state = SCANNING # Go back to scanning state.
            return # Exit tracking function.

        # --- Placeholder for more advanced tracking logic ---
        # With only a TFmini-S, precise angular tracking (left/right, up/down) is challenging.
        # The TFmini-S provides distance, but not the exact angular deviation from the center
        # of its beam.
        # For this setup, "tracking" primarily means staying pointed at the last known
        # target angle and re-scanning if the drone moves out of the narrow beam.

        # For now, the servo and stepper will simply hold their last known target positions.
        set_servo_angle(target_pan_angle)
        # Stepper movement for tilt (if needed, based on a hypothetical tilt sensor or scan)
        # For this example, tilt is static after initial detection.
        # If you want to add tilt scanning, you'd need to extend the scan_for_drone logic
        # or implement a separate tilt adjustment.

        time.sleep(0.1) # Small delay to control the tracking loop frequency.
    else:
        # If no valid TFmini-S data is received for a certain period, assume drone is lost.
        if time.time() - track_drone.last_read_time > 1.5: # 1.5 second timeout.
            send_message("No TFmini-S data for a while. Drone potentially lost. Re-scanning...")
            current_state = SCANNING

# --- Main Program Setup and Loop ---
def setup():
    """Initializes GPIO, servo, stepper, and TFmini-S."""
    global pan_servo_pwm

    send_message("System Booting...")

    # --- GPIO Setup ---
    GPIO.setmode(GPIO.BCM) # Use BCM numbering for GPIO pins
    GPIO.setwarnings(False) # Disable warnings for now, but be careful in production

    # --- Servo Setup ---
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    # Start PWM on servo pin at 50Hz (standard for hobby servos)
    pan_servo_pwm = GPIO.PWM(SERVO_PIN, 50)
    pan_servo_pwm.start(0) # Start with 0 duty cycle (off)
    set_servo_angle(target_pan_angle) # Move servo to its initial center position.

    # --- Stepper Motor Setup (DRV8825) ---
    # Set stepper driver control pins as outputs.
    GPIO.setup(STEPPER_DIR_PIN, GPIO.OUT)
    GPIO.setup(STEPPER_STEP_PIN, GPIO.OUT)
    GPIO.setup(STEPPER_ENABLE_PIN, GPIO.OUT)
    GPIO.setup(STEPPER_MS1_PIN, GPIO.OUT)
    GPIO.setup(STEPPER_MS2_PIN, GPIO.OUT)
    GPIO.setup(STEPPER_MS3_PIN, GPIO.OUT)

    # Enable the stepper driver. DRV8825 is enabled when its ENABLE pin is LOW.
    GPIO.output(STEPPER_ENABLE_PIN, GPIO.LOW)
    # Set the microstepping resolution for the DRV8825.
    set_stepper_microstepping(MICROSTEPS)

    # --- TFmini-S Setup ---
    setup_tfmini_s()

    send_message("System Ready. Scanning for drone...")

def loop():
    """The main loop continuously checks the current state and calls the appropriate function."""
    while True:
        if current_state == SCANNING:
            scan_for_drone() # Execute scanning logic.
        elif current_state == TRACKING:
            track_drone() # Execute tracking logic.
        time.sleep(0.01) # Small delay to prevent busy-waiting

def cleanup():
    """Cleans up GPIO and serial resources on exit."""
    send_message("Cleaning up resources...")
    if pan_servo_pwm:
        pan_servo_pwm.stop()
    GPIO.cleanup() # Resets all GPIO pins to default state
    if tfmini_serial:
        tfmini_serial.close()
    send_message("Cleanup complete. Exiting.")

# --- Main execution block ---
if __name__ == "__main__":
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        cleanup()
    except Exception as e:
        send_message(f"An error occurred: {e}")
        cleanup()