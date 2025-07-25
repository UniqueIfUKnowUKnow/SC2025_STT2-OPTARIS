// Include necessary libraries
// The Servo library is standard with Arduino IDE.
#include <Servo.h>
#include <TFminiS.h>

// For the TFmini-S, a dedicated library is highly recommended for robust communication.
// You can install "TFmini_S" by Buddydabuddy via the Arduino Library Manager.
// For this example, a basic manual UART parsing is included, but it's less robust.
// If you install the library, you would typically include it like this:
// #include <TFmini_S.h>

// --- Pin Definitions ---
// MG90S Servo (Pan/Horizontal control)
// Connect servo signal pin to Arduino digital pin 9.
#define SERVO_PIN 9

// DRV8825 Stepper Motor Driver (Tilt/Vertical control)
// Connect DRV8825 DIR pin to Arduino digital pin 5.
#define STEPPER_DIR_PIN 5
// Connect DRV8825 STEP pin to Arduino digital pin 6.
#define STEPPER_STEP_PIN 6
// Connect DRV8825 ENABLE pin to Arduino digital pin 7.
// Set LOW to enable the driver, HIGH to disable.
#define STEPPER_ENABLE_PIN 7
// Microstepping pins (MS1, MS2, MS3) on DRV8825.
// These determine the microstepping resolution (e.g., 1/16, 1/32).
#define STEPPER_MS1_PIN 2
#define STEPPER_MS2_PIN 3
#define STEPPER_MS3_PIN 4

// TFmini-S LiDAR sensor (UART communication)
// Arduino R4 Minima has multiple hardware serial ports.
// Using Serial1 for TFmini-S is recommended for reliability and speed.
// Connect TFmini-S TX to Arduino RX1 (pin 0).
// Connect TFmini-S RX to Arduino TX1 (pin 1).
#define TFMINI_SERIAL Serial1 // Using Hardware Serial1

// --- Global Objects ---
Servo panServo; // Servo object for horizontal (pan) movement

// --- Stepper Motor Configuration ---
// STEPS_PER_REVOLUTION: Number of full steps for one revolution of your stepper motor.
// Common values are 200 (for 1.8 degree/step) or 400 (for 0.9 degree/step).
const int STEPS_PER_REVOLUTION = 200;
// MICROSTEPS: The microstepping setting for your DRV8825 driver.
// This example assumes 1/16 microstepping, which is common.
// Ensure your MS1, MS2, MS3 pins are configured accordingly on the DRV8825.
const int MICROSTEPS = 16;
// STEPS_PER_DEGREE_TILT: Calculated steps needed to move the stepper by one degree.
// This is a theoretical value. Actual physical gearing will affect this.
const int STEPS_PER_DEGREE_TILT = (STEPS_PER_REVOLUTION * MICROSTEPS) / 360;

// --- TFmini-S Data Variables ---
int distance = 0; // Distance reading from TFmini-S in centimeters.
int strength = 0; // Signal strength reading from TFmini-S.

// --- Tracking State Variables ---
// Defines the different states of the drone tracking system.
enum TrackingState {
  SCANNING, // Actively searching for a drone.
  TRACKING  // Drone detected, actively following it.
};
TrackingState currentState = SCANNING; // Initial state is SCANNING.

// Drone target position (angles)
// These variables store the last known or desired pan/tilt angles of the drone.
int targetPanAngle = 90; // Initial center position for servo (0-180 degrees).
// For tilt, we'll use a conceptual angle, as the stepper's range might vary.
// A value like 90 could represent a horizontal tilt.
int targetTiltAngle = 90;

// Scan parameters
// Defines the range and step size for the horizontal scanning motion.
const int SCAN_START_PAN = 45;  // Starting pan angle for scanning (degrees).
const int SCAN_END_PAN = 135;   // Ending pan angle for scanning (degrees).
const int SCAN_STEP_PAN = 5;    // Increment/decrement step size for pan during scan (degrees).
const int SCAN_DELAY_MS = 50;   // Delay after each scan step to allow servo to move and sensor to read.

// Drone detection range (in centimeters)
// The system will consider an object a "drone" if its distance falls within this range.
const int DETECTION_RANGE_MIN = 800;  // Minimum distance for detection (8 meters).
const int DETECTION_RANGE_MAX = 1200; // Maximum distance for detection (12 meters).
// Minimum signal strength for a valid detection.
const int MIN_STRENGTH = 100;

// --- Function Prototypes ---
// Declares functions before they are defined, so the compiler knows about them.
void setupTFminiS();
bool readTFminiS();
void setStepperMicrostepping(int ms);
void moveStepper(int steps, int direction);
void scanForDrone();
void trackDrone();
void sendMessage(const String& message); // Custom function to print messages to Serial Monitor.

void setup() {
  // Initialize Serial communication for debugging and messages.
  Serial.begin(115200);
  sendMessage("System Booting...");

  // --- Servo Setup ---
  // Attaches the servo object to the specified pin.
  panServo.attach(SERVO_PIN);
  // Moves the servo to its initial center position.
  panServo.write(targetPanAngle);

  // --- Stepper Motor Setup (DRV8825) ---
  // Set stepper driver control pins as outputs.
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  pinMode(STEPPER_MS1_PIN, OUTPUT);
  pinMode(STEPPER_MS2_PIN, OUTPUT);
  pinMode(STEPPER_MS3_PIN, OUTPUT);

  // Enable the stepper driver. DRV8825 is enabled when its ENABLE pin is LOW.
  digitalWrite(STEPPER_ENABLE_PIN, LOW);
  // Set the microstepping resolution for the DRV8825.
  setStepperMicrostepping(MICROSTEPS);

  // --- TFmini-S Setup ---
  // Initializes the TFmini-S sensor communication.
  setupTFminiS();

  sendMessage("System Ready. Scanning for drone...");
}

void loop() {
  // The main loop continuously checks the current state and calls the appropriate function.
  switch (currentState) {
    case SCANNING:
      scanForDrone(); // Execute scanning logic.
      break;
    case TRACKING:
      trackDrone(); // Execute tracking logic.
      break;
  }
}

// --- TFmini-S Functions ---

// Initializes the hardware serial port for TFmini-S communication.
void setupTFminiS() {
  TFMINI_SERIAL.begin(115200); // TFmini-S default baud rate is 115200.
  sendMessage("TFmini-S initialized.");
}

// Reads data from TFmini-S via UART.
// This function parses the 9-byte data frame from the TFmini-S.
// Returns true if a valid frame with correct checksum is received, false otherwise.
bool readTFminiS() {
  static byte frame[9]; // Buffer to store the 9-byte data frame.
  static int i = 0;     // Index for the current byte in the frame.

  // Read available bytes from the serial buffer.
  while (TFMINI_SERIAL.available()) {
    byte data = TFMINI_SERIAL.read();

    if (i == 0) { // First byte should be 0x59 (start of frame).
      if (data == 0x59) {
        frame[i++] = data;
      }
    } else if (i == 1) { // Second byte should also be 0x59.
      if (data == 0x59) {
        frame[i++] = data;
      } else { // If not 0x59, reset frame index.
        i = 0;
      }
    } else { // Store subsequent bytes until the frame is complete.
      frame[i++] = data;
      if (i == 9) { // Full 9-byte frame received.
        // Extract distance (bytes 2 and 3) and strength (bytes 4 and 5).
        int dist = frame[2] + (frame[3] << 8); // Distance LSB + MSB.
        int str = frame[4] + (frame[5] << 8);  // Strength LSB + MSB.
        int checksum = frame[8];               // Checksum byte.

        // Calculate checksum for bytes 0-7.
        int calculatedChecksum = 0;
        for (int j = 0; j < 8; j++) {
          calculatedChecksum += frame[j];
        }

        // Check if calculated checksum matches the received checksum.
        if (calculatedChecksum == checksum) {
          distance = dist;    // Update global distance variable.
          strength = str;     // Update global strength variable.
          i = 0;              // Reset index for the next frame.
          return true;        // Valid frame received.
        } else {
          // Checksum mismatch, discard frame and reset.
          sendMessage("TFmini-S checksum error.");
          i = 0;
        }
      }
    }
  }
  return false; // No valid frame received yet.
}

// --- Stepper Motor Functions ---

// Sets the microstepping resolution for the DRV8825 driver.
// `ms`: Desired microstepping value (1, 2, 4, 8, 16, 32).
// This function configures the MS1, MS2, MS3 pins accordingly.
void setStepperMicrostepping(int ms) {
  // Reset all microstepping pins to LOW initially.
  digitalWrite(STEPPER_MS1_PIN, LOW);
  digitalWrite(STEPPER_MS2_PIN, LOW);
  digitalWrite(STEPPER_MS3_PIN, LOW);

  // Set pins based on the desired microstepping value.
  switch (ms) {
    case 1: // Full step (all MS pins LOW)
      break;
    case 2: // 1/2 step
      digitalWrite(STEPPER_MS1_PIN, HIGH);
      break;
    case 4: // 1/4 step
      digitalWrite(STEPPER_MS2_PIN, HIGH);
      break;
    case 8: // 1/8 step
      digitalWrite(STEPPER_MS1_PIN, HIGH);
      digitalWrite(STEPPER_MS2_PIN, HIGH);
      break;
    case 16: // 1/16 step
      digitalWrite(STEPPER_MS3_PIN, HIGH);
      break;
    case 32: // 1/32 step
      digitalWrite(STEPPER_MS1_PIN, HIGH);
      digitalWrite(STEPPER_MS3_PIN, HIGH);
      break;
    default:
      sendMessage("Invalid microstepping value. Defaulting to 1 (Full Step).");
      break;
  }
  delay(1); // Small delay to allow pin states to settle.
}

// Moves the stepper motor by a given number of steps in a specified direction.
// `steps`: The number of microsteps to move.
// `direction`: 0 for one direction, 1 for the other (depends on wiring).
void moveStepper(int steps, int direction) {
  digitalWrite(STEPPER_DIR_PIN, direction); // Set the direction.
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEPPER_STEP_PIN, HIGH); // Pulse the STEP pin HIGH.
    delayMicroseconds(500); // Adjust this delay for desired speed (lower value = faster).
    digitalWrite(STEPPER_STEP_PIN, LOW);  // Pulse the STEP pin LOW.
    delayMicroseconds(500); // Adjust this delay for desired speed.
  }
}

// --- Drone Tracking Logic ---

// Scans the environment by moving the servo and reading the TFmini-S.
void scanForDrone() {
  static int currentPan = SCAN_START_PAN; // Current pan angle during scanning.
  static int panDirection = 1; // 1 for increasing angle, -1 for decreasing.

  panServo.write(currentPan); // Move servo to the current scan position.
  delay(SCAN_DELAY_MS);       // Wait for the servo to reach its position.

  if (readTFminiS()) { // Attempt to read distance from TFmini-S.
    Serial.print("Scanning at Pan: ");
    Serial.print(currentPan);
    Serial.print(" deg, Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Strength: ");
    Serial.println(strength);

    // Check if an object is within the detection range and has sufficient signal strength.
    if (distance >= DETECTION_RANGE_MIN && distance <= DETECTION_RANGE_MAX && strength >= MIN_STRENGTH) {
      sendMessage("Drone detected!");
      targetPanAngle = currentPan; // Store the pan angle where the drone was detected.
      // For tilt, we assume the initial tilt is adequate for detection.
      // In a real system, you might also scan vertically with the stepper.
      currentState = TRACKING; // Transition to the TRACKING state.
      return; // Exit the scan function to start tracking.
    }
  }

  // Move to the next pan position for scanning.
  currentPan += (SCAN_STEP_PAN * panDirection);

  // Reverse scanning direction if limits are reached.
  if (currentPan >= SCAN_END_PAN) {
    panDirection = -1;
    currentPan = SCAN_END_PAN; // Ensure it doesn't go beyond the end limit.
  } else if (currentPan <= SCAN_START_PAN) {
    panDirection = 1;
    currentPan = SCAN_START_PAN; // Ensure it doesn't go below the start limit.
  }
}

// Tracks the drone once it has been detected.
// This implementation is basic due to the limitations of a single distance sensor for angular tracking.
void trackDrone() {
  // Static variable to keep track of the last time TFmini-S data was successfully read.
  static unsigned long lastReadTime = millis();

  if (readTFminiS()) { // Continuously read distance while tracking.
    lastReadTime = millis(); // Update last read time.
    Serial.print("Tracking - Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Strength: ");
    Serial.println(strength);

    // Check if the drone is still within the detection range and has sufficient strength.
    // If not, it means the drone has moved out of the sensor's narrow beam or is too far/close.
    if (distance < DETECTION_RANGE_MIN || distance > DETECTION_RANGE_MAX || strength < MIN_STRENGTH) {
      sendMessage("Drone lost! Re-scanning...");
      currentState = SCANNING; // Go back to scanning state.
      return; // Exit tracking function.
    }

    // --- Placeholder for more advanced tracking logic ---
    // With only a TFmini-S, precise angular tracking (left/right, up/down) is challenging.
    // The TFmini-S provides distance, but not the exact angular deviation from the center
    // of its beam.
    // For this setup, "tracking" primarily means staying pointed at the last known
    // target angle and re-scanning if the drone moves out of the narrow beam.

    // If you had a camera or an array of sensors, you would calculate an "error"
    // (e.g., how far off-center the drone is) and use that to adjust the servo and stepper.
    // Example (conceptual):
    // int panError = calculatePanError(dronePosition); // Requires a vision system or similar
    // panServo.write(panServo.read() + panError * Kp_pan); // Kp_pan is a proportional gain
    // int tiltError = calculateTiltError(dronePosition);
    // moveStepper(abs(tiltError * Kp_tilt), (tiltError > 0 ? 1 : 0));

    // For now, the servo and stepper will simply hold their last known target positions.
    panServo.write(targetPanAngle);
    // Stepper movement for tilt (if needed, based on a hypothetical tilt sensor or scan)
    // For this example, tilt is static after initial detection.
    // If you want to add tilt scanning, you'd need to extend the scanForDrone logic
    // or implement a separate tilt adjustment.

    delay(100); // Small delay to control the tracking loop frequency.
  } else {
    // If no valid TFmini-S data is received for a certain period, assume drone is lost.
    if (millis() - lastReadTime > 1500) { // 1.5 second timeout.
      sendMessage("No TFmini-S data for a while. Drone potentially lost. Re-scanning...");
      currentState = SCANNING;
    }
  }
}

// Custom function to print messages to the Serial Monitor.
// This acts as a simple message box for user feedback.
void sendMessage(const String& message) {
  Serial.println("MESSAGE: " + message);
}
