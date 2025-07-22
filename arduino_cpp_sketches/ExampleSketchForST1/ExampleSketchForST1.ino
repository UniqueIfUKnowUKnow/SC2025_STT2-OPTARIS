// Arduino Drone Tracker Sketch
// This sketch controls a TFmini-S LiDAR, an MG90S servo, and a stepper motor
// with a DRV8825 driver to find and track an object (e.g., a drone)
// approximately 10 meters away.

// --- Libraries ---
// Servo library for MG90S servo control
#include <Servo.h>
// AccelStepper library for DRV8825 and stepper motor control
// Install from Arduino IDE: Sketch -> Include Library -> Manage Libraries... -> Search for "AccelStepper"
#include <AccelStepper.h>
// SoftwareSerial for TFmini-S LiDAR communication (if using non-hardware serial pins)
#include <SoftwareSerial.h>

// --- Pin Definitions ---
// TFmini-S LiDAR (using SoftwareSerial for flexibility, adjust pins as needed)
#define TFMINI_RX_PIN 2 // Connect TFmini-S TX to Arduino D2
#define TFMINI_TX_PIN 3 // Connect TFmini-S RX to Arduino D3

// DRV8825 Stepper Motor Driver
#define STEPPER_DIR_PIN 4  // Direction pin for DRV8825
#define STEPPER_STEP_PIN 5 // Step pin for DRV8825
#define STEPPER_ENABLE_PIN 6 // Enable pin for DRV8825 (active low, connect to GND if not used)
// Microstepping pins (MS1, MS2, MS3) are typically set by jumpers on the DRV8825 board.
// For simplicity, we assume full-step mode (all MS pins low or floating, depending on board).

// MG90S Servo
#define SERVO_PIN 9 // PWM pin for MG90S servo

// --- Global Objects ---
SoftwareSerial tfminiSerial(TFMINI_RX_PIN, TFMINI_TX_PIN); // SoftwareSerial object for TFmini-S
Servo panServo; // Servo object for pan (horizontal) movement (MG90S for tilt in this setup, let's rename for clarity)
// Let's assume MG90S is for TILT (vertical) and Stepper is for PAN (horizontal)
Servo tiltServo; // MG90S servo for vertical (tilt) movement

// AccelStepper object for the stepper motor.
// AccelStepper::DRIVER mode: (1 for DRIVER, STEP/DIR pins)
// AccelStepper(mode, stepPin, dirPin)
AccelStepper stepper(AccelStepper::DRIVER, STEPPER_STEP_PIN, STEPPER_DIR_PIN);

// --- Constants and Configuration ---
// TFmini-S related
const int TFMINI_BAUDRATE = 115200; // TFmini-S default baud rate
const int MAX_DISTANCE_CM = 1000;   // Max tracking distance (10 meters = 1000 cm)
const int MIN_DISTANCE_CM = 50;     // Min distance to avoid tracking ground/very close objects (50 cm)
const int DISTANCE_READ_INTERVAL_MS = 50; // How often to read TFmini-S data

// Stepper Motor (PAN) configuration
const long STEPPER_MAX_SPEED = 2000; // Max speed in steps/second
const long STEPPER_ACCELERATION = 1000; // Acceleration in steps/second^2
const int STEPS_PER_DEGREE = 10; // Adjust based on your stepper motor and microstepping settings
                                  // (e.g., 200 steps/revolution for 1.8 degree motor, 1.8 * 10 = 18 steps/degree for full step)
const int PAN_RANGE_DEGREES = 180; // Total horizontal pan range (e.g., 180 degrees)
const int PAN_CENTER_STEPS = (PAN_RANGE_DEGREES / 2) * STEPS_PER_DEGREE; // Center position in steps

// Servo Motor (TILT) configuration
const int SERVO_MIN_ANGLE = 0;   // Minimum tilt angle (degrees)
const int SERVO_MAX_ANGLE = 180; // Maximum tilt angle (degrees)
const int SERVO_CENTER_ANGLE = 90; // Center tilt angle (degrees)
const int TILT_SWEEP_INCREMENT = 5; // Degrees to move during vertical sweep

// Tracking parameters
const int TRACKING_THRESHOLD_CM = 50; // How much distance can vary before re-adjusting
const int ANGULAR_ADJUSTMENT_STEPS = 5; // Small angular adjustment steps for tracking
const int ANGULAR_ADJUSTMENT_DEGREES = 1; // Small angular adjustment for servo

// State machine for drone tracking
enum TrackingState {
  SEARCHING,
  TRACKING
};
TrackingState currentState = SEARCHING;

// --- Variables ---
unsigned long lastDistanceReadTime = 0;
int currentDistanceCm = 0; // Current distance from TFmini-S
int currentTiltAngle = SERVO_CENTER_ANGLE;
int currentPanSteps = PAN_CENTER_STEPS;
int panSweepDirection = 1; // 1 for right, -1 for left
int tiltSweepDirection = 1; // 1 for up, -1 for down

// --- Function Prototypes ---
void readTFminiData();
bool isValidTarget(int distance);
void enterSearchingMode();
void searchForDrone();
void enterTrackingMode();
void trackDrone();
void moveStepperTo(long targetSteps);
void moveServoTo(int targetAngle);
void messageBox(String message); // Custom message box function

// --- Setup Function ---
void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(115200);
  Serial.println("Drone Tracker Initializing...");

  // Initialize TFmini-S SoftwareSerial
  tfminiSerial.begin(TFMINI_BAUDRATE);
  Serial.println("TFmini-S Serial initialized.");

  // Attach servo to pin
  tiltServo.attach(SERVO_PIN);
  tiltServo.write(SERVO_CENTER_ANGLE); // Move servo to center
  Serial.print("Servo attached to pin ");
  Serial.print(SERVO_PIN);
  Serial.print(", set to ");
  Serial.print(SERVO_CENTER_ANGLE);
  Serial.println(" degrees.");

  // Configure stepper motor driver enable pin
  pinMode(STEPPER_ENABLE_PIN, OUTPUT);
  digitalWrite(STEPPER_ENABLE_PIN, LOW); // Enable DRV8825 (active low)
  Serial.println("DRV8825 enabled.");

  // Configure AccelStepper
  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACCELERATION);
  stepper.setCurrentPosition(PAN_CENTER_STEPS); // Set current position as center
  stepper.moveTo(PAN_CENTER_STEPS); // Move to center position
  Serial.print("Stepper initialized, set to ");
  Serial.print(PAN_CENTER_STEPS);
  Serial.println(" steps (center).");

  // Initial state
  currentState = SEARCHING;
  Serial.println("Entering SEARCHING mode.");
  messageBox("System Ready! Searching for drone...");
}

// --- Main Loop ---
void loop() {
  // Always read TFmini-S data in the background
  readTFminiData();

  // Run the state machine
  switch (currentState) {
    case SEARCHING:
      searchForDrone();
      break;
    case TRACKING:
      trackDrone();
      break;
  }

  // Allow AccelStepper to run
  stepper.run();
}

// --- Function Implementations ---

/**
 * @brief Reads distance data from the TFmini-S LiDAR sensor.
 * This function parses the serial data stream from the TFmini-S
 * and updates the `currentDistanceCm` variable.
 * The TFmini-S sends data in a specific frame format.
 */
void readTFminiData() {
  if (millis() - lastDistanceReadTime < DISTANCE_READ_INTERVAL_MS) {
    return; // Don't read too frequently
  }
  lastDistanceReadTime = millis();

  static byte buffer[9]; // TFmini-S data frame is 9 bytes
  static int i = 0;

  while (tfminiSerial.available()) {
    byte byteRead = tfminiSerial.read();

    if (i == 0) { // First byte of the frame (0x59)
      if (byteRead == 0x59) {
        buffer[i++] = byteRead;
      }
    } else if (i == 1) { // Second byte of the frame (0x59)
      if (byteRead == 0x59) {
        buffer[i++] = byteRead;
      } else {
        i = 0; // Reset if second byte is not 0x59
      }
    } else { // Remaining bytes
      buffer[i++] = byteRead;
      if (i == 9) { // Full frame received
        // Calculate checksum
        int checksum = 0;
        for (int j = 0; j < 8; j++) {
          checksum += buffer[j];
        }
        if ((checksum & 0xFF) == buffer[8]) { // Checksum matches
          int dist = buffer[2] + (buffer[3] << 8); // Distance in cm
          // int strength = buffer[4] + (buffer[5] << 8); // Signal strength (optional)

          currentDistanceCm = dist;
          // Serial.print("Distance: ");
          // Serial.print(currentDistanceCm);
          // Serial.println(" cm");
        } else {
          Serial.println("TFmini-S Checksum error!");
        }
        i = 0; // Reset buffer index for next frame
      }
    }
  }
}

/**
 * @brief Checks if the given distance represents a valid target.
 * A valid target is within the specified min/max distance range.
 * @param distance The distance in centimeters.
 * @return True if it's a valid target, false otherwise.
 */
bool isValidTarget(int distance) {
  // Check if distance is within the valid range and not 0 (which often indicates no reading)
  return (distance > MIN_DISTANCE_CM && distance <= MAX_DISTANCE_CM);
}

/**
 * @brief Transitions the system to SEARCHING mode.
 * Resets motor positions and prepares for scanning.
 */
void enterSearchingMode() {
  currentState = SEARCHING;
  Serial.println("Entering SEARCHING mode.");
  messageBox("Searching for drone...");
  // Reset pan to center and start sweeping
  stepper.moveTo(PAN_CENTER_STEPS);
  currentPanSteps = PAN_CENTER_STEPS;
  panSweepDirection = 1; // Start sweeping right
  // Reset tilt to center
  tiltServo.write(SERVO_CENTER_ANGLE);
  currentTiltAngle = SERVO_CENTER_ANGLE;
  tiltSweepDirection = 1; // Start sweeping up
}

/**
 * @brief Implements the drone searching logic.
 * Sweeps the stepper motor horizontally and adjusts the servo vertically.
 * Transitions to TRACKING mode if a valid target is found.
 */
void searchForDrone() {
  // Horizontal sweep (Pan)
  if (stepper.distanceToGo() == 0) {
    currentPanSteps += (PAN_RANGE_DEGREES / 2) * STEPS_PER_DEGREE * panSweepDirection;
    if (currentPanSteps > PAN_RANGE_DEGREES * STEPS_PER_DEGREE || currentPanSteps < 0) {
      panSweepDirection *= -1; // Reverse direction
      currentPanSteps = PAN_CENTER_STEPS + (PAN_RANGE_DEGREES / 2) * STEPS_PER_DEGREE * panSweepDirection; // Adjust to stay within bounds
      // After a full horizontal sweep, adjust tilt
      currentTiltAngle += TILT_SWEEP_INCREMENT * tiltSweepDirection;
      if (currentTiltAngle > SERVO_MAX_ANGLE || currentTiltAngle < SERVO_MIN_ANGLE) {
        tiltSweepDirection *= -1; // Reverse tilt direction
        currentTiltAngle = SERVO_CENTER_ANGLE + TILT_SWEEP_INCREMENT * tiltSweepDirection; // Adjust to stay within bounds
      }
      moveServoTo(currentTiltAngle);
    }
    stepper.moveTo(currentPanSteps);
  }

  // Check for target while sweeping
  if (isValidTarget(currentDistanceCm)) {
    Serial.print("Target found at ");
    Serial.print(currentDistanceCm);
    Serial.print(" cm. Pan: ");
    Serial.print(stepper.currentPosition() / STEPS_PER_DEGREE);
    Serial.print(" deg, Tilt: ");
    Serial.print(tiltServo.read());
    Serial.println(" deg.");
    enterTrackingMode();
  }
}

/**
 * @brief Transitions the system to TRACKING mode.
 * Stops sweeping and prepares for fine adjustments.
 */
void enterTrackingMode() {
  currentState = TRACKING;
  Serial.println("Entering TRACKING mode.");
  messageBox("Drone detected! Tracking...");
  // Stop current motor movements
  stepper.stop();
  stepper.setCurrentPosition(stepper.currentPosition()); // Reset position to current
}

/**
 * @brief Implements the drone tracking logic.
 * Makes small adjustments to keep the detected object centered.
 * Transitions back to SEARCHING mode if the target is lost.
 */
void trackDrone() {
  // If target is lost, go back to searching
  if (!isValidTarget(currentDistanceCm)) {
    Serial.println("Target lost. Re-entering SEARCHING mode.");
    enterSearchingMode();
    return;
  }

  // Basic tracking logic: Try to keep the target centered by finding the minimum distance.
  // This is a simplified "hill-climbing" approach for a single-point LiDAR.

  // Store current best position and distance
  static int bestDistance = currentDistanceCm;
  static long bestPanSteps = stepper.currentPosition();
  static int bestTiltAngle = tiltServo.read();

  // Try small adjustments to find the "center" of the target
  // Check slightly left, right, up, down to find the minimum distance
  // This is a very basic approach and can be improved with more advanced algorithms.

  // Try adjusting pan
  long originalPan = stepper.currentPosition();
  int originalTilt = tiltServo.read();

  // Test slightly left
  moveStepperTo(originalPan - ANGULAR_ADJUSTMENT_STEPS);
  stepper.runToPosition(); // Block until move complete for testing
  readTFminiData(); // Get new distance
  if (isValidTarget(currentDistanceCm) && currentDistanceCm < bestDistance) {
    bestDistance = currentDistanceCm;
    bestPanSteps = stepper.currentPosition();
  }

  // Test slightly right
  moveStepperTo(originalPan + ANGULAR_ADJUSTMENT_STEPS);
  stepper.runToPosition();
  readTFminiData();
  if (isValidTarget(currentDistanceCm) && currentDistanceCm < bestDistance) {
    bestDistance = currentDistanceCm;
    bestPanSteps = stepper.currentPosition();
  }

  // Move back to best pan position found
  moveStepperTo(bestPanSteps);
  stepper.runToPosition();

  // Test slightly up
  moveServoTo(originalTilt + ANGULAR_ADJUSTMENT_DEGREES);
  delay(50); // Give servo time to move
  readTFminiData();
  if (isValidTarget(currentDistanceCm) && currentDistanceCm < bestDistance) {
    bestDistance = currentDistanceCm;
    bestTiltAngle = tiltServo.read();
  }

  // Test slightly down
  moveServoTo(originalTilt - ANGULAR_ADJUSTMENT_DEGREES);
  delay(50);
  readTFminiData();
  if (isValidTarget(currentDistanceCm) && currentDistanceCm < bestDistance) {
    bestDistance = currentDistanceCm;
    bestTiltAngle = tiltServo.read();
  }

  // Move back to best tilt position found
  moveServoTo(bestTiltAngle);

  // If the current distance is significantly different from the "best" distance found in this micro-scan,
  // it means the target might have moved, or we're off-center.
  // Re-center the motors to the best found position.
  if (abs(currentDistanceCm - bestDistance) > TRACKING_THRESHOLD_CM) {
    Serial.print("Adjusting to best position. Dist: ");
    Serial.print(bestDistance);
    Serial.print("cm. Pan: ");
    Serial.print(bestPanSteps / STEPS_PER_DEGREE);
    Serial.print("deg, Tilt: ");
    Serial.print(bestTiltAngle);
    Serial.println("deg.");
    moveStepperTo(bestPanSteps);
    moveServoTo(bestTiltAngle);
  }

  // Keep the stepper running to maintain position or complete small adjustments
  stepper.run();
}

/**
 * @brief Moves the stepper motor to a target position in steps.
 * @param targetSteps The absolute target position in steps.
 */
void moveStepperTo(long targetSteps) {
  // Ensure target steps are within bounds
  targetSteps = constrain(targetSteps, 0, PAN_RANGE_DEGREES * STEPS_PER_DEGREE);
  if (stepper.targetPosition() != targetSteps) {
    stepper.moveTo(targetSteps);
    Serial.print("Moving stepper to: ");
    Serial.print(targetSteps);
    Serial.println(" steps.");
  }
}

/**
 * @brief Moves the servo motor to a target angle.
 * @param targetAngle The absolute target angle in degrees (0-180).
 */
void moveServoTo(int targetAngle) {
  // Ensure target angle is within bounds
  targetAngle = constrain(targetAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  if (tiltServo.read() != targetAngle) {
    tiltServo.write(targetAngle);
    Serial.print("Moving servo to: ");
    Serial.print(targetAngle);
    Serial.println(" degrees.");
  }
}

/**
 * @brief A simple message box function for serial output,
 * replacing `alert()` for better user experience in an embedded context.
 * @param message The string message to display.
 */
void messageBox(String message) {
  Serial.print("MESSAGE: ");
  Serial.println(message);
}
