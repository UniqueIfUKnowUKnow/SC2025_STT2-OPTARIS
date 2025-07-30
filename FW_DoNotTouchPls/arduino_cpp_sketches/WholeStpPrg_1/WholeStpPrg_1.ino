#include <AccelStepper.h>
#include <Servo.h>

// --- Stepper Motor Definitions (Now for Horizontal Sweep) ---
const int DIR_PIN = 2;    // Digital pin connected to DRV8825 DIR pin
const int STEP_PIN = 3;   // Digital pin connected to DRV8825 STEP pin
const int ENABLE_PIN = 4; // Digital pin connected to DRV8825 EN pin (LOW to enable, HIGH to disable)
const int STEPS_PER_REVOLUTION = 200; // IMPORTANT: Adjust for microstepping (e.g., 200 * 16 = 3200 for 1/16 microsteps)

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --- Servo Motor Definitions (Now for Vertical Indexing) ---
Servo myServo;
const int servoPin = 9;

// --- Lidar and Scan Parameters ---
const int LIDAR_FOV_DEGREES = 2; // Lidar sensor's Field of View (applies to both axes)

// Stepper scan parameters (Horizontal Sweep: 180 degrees back and forth)
const long HORIZONTAL_SWEEP_RANGE_STEPS = (long)STEPS_PER_REVOLUTION / 2; // 180 degrees rotation (e.g., 100 steps for 200 step/rev motor)
// If using microstepping, adjust STEPS_PER_REVOLUTION first, then calculate.
// E.g., for 1/16 microsteps: (200 * 16) / 2 = 1600 steps for 180 degrees.

// Servo scan parameters (Vertical Indexing: 0 to 180 degrees in FOV increments)
const int SERVO_START_ANGLE = 0;
const int SERVO_END_ANGLE = 180;
const int SERVO_INCREMENT_DEGREES = LIDAR_FOV_DEGREES; // Servo advances by Lidar FOV
const int SERVO_MOVE_SPEED_MS = 50; // Time in ms for servo to move to next vertical position (adjust for smoothness)

// Calculated number of vertical rows
const int NUMBER_OF_VERTICAL_ROWS = (SERVO_END_ANGLE - SERVO_START_ANGLE) / SERVO_INCREMENT_DEGREES;

// --- Control Variables for Asynchronous Operations ---
unsigned long lastServoMoveMillis = 0;
int currentServoAngle = SERVO_START_ANGLE;
int currentVerticalRowIndex = 0; // Tracks which vertical row (0 to NUMBER_OF_VERTICAL_ROWS)

bool stepperSweepingRight = true; // True for 0 to HORIZONTAL_SWEEP_RANGE_STEPS, false for back

// State machine variables
enum ScanState {
  INITIALIZING,
  STEPPER_HORIZONTAL_SWEEP,
  SERVO_VERTICAL_ADVANCE,
  SCAN_COMPLETE_RESET
};
ScanState currentScanState = INITIALIZING;

void setup() {
  Serial.begin(9600);
  Serial.println("Combined Stepper (Horizontal) and Servo (Vertical) Lidar Scan Start");

  // --- Stepper Setup ---
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable the stepper driver
  Serial.println("Stepper Driver Enabled.");

  stepper.setMaxSpeed(1000);    // Adjust max speed (steps/second) for horizontal sweep
  stepper.setAcceleration(500); // Adjust acceleration (steps/second^2)
  stepper.setCurrentPosition(0); // Ensure stepper starts at known home position

  // --- Servo Setup ---
  myServo.attach(servoPin);
  myServo.write(SERVO_START_ANGLE); // Initialize servo to the top/bottom of its range
  delay(1000); // Give servo time to reach initial position
  Serial.println("Servo Initialized.");

  currentScanState = STEPPER_HORIZONTAL_SWEEP; // Start by sweeping the first horizontal line
  Serial.println("Starting Zigzag Scan Cycle.");

  Serial.print("Calculated HORIZONTAL_SWEEP_RANGE_STEPS: ");
  Serial.println(HORIZONTAL_SWEEP_RANGE_STEPS);
  Serial.print("Calculated NUMBER_OF_VERTICAL_ROWS: ");
  Serial.println(NUMBER_OF_VERTICAL_ROWS);
}

void loop() {
  // Always call stepper.run() to allow it to move
  stepper.run();

  unsigned long currentMillis = millis();

  switch (currentScanState) {
    case INITIALIZING:
      currentScanState = STEPPER_HORIZONTAL_SWEEP;
      Serial.println("State: STEPPER_HORIZONTAL_SWEEP (Initial)");
      break;

    case STEPPER_HORIZONTAL_SWEEP:
      // Command the stepper to move if it's not already moving
      if (!stepper.isRunning()) {
        long targetStepperPos;
        if (stepperSweepingRight) {
          targetStepperPos = HORIZONTAL_SWEEP_RANGE_STEPS;
        } else {
          targetStepperPos = 0; // Back to home (leftmost) position
        }
        stepper.moveTo(targetStepperPos);
        Serial.print("Commanding Stepper to: ");
        Serial.println(targetStepperPos);
      }

      // If stepper reaches its target, it means a full horizontal sweep is done
      if (stepper.currentPosition() == stepper.targetPosition() && !stepper.isRunning()) {
        Serial.println("Stepper horizontal sweep complete.");
        stepperSweepingRight = !stepperSweepingRight; // Toggle direction for next horizontal sweep

        currentVerticalRowIndex++; // Move to the next vertical row

        if (currentVerticalRowIndex <= NUMBER_OF_VERTICAL_ROWS) {
          // Stepper finished, now advance servo vertically
          currentScanState = SERVO_VERTICAL_ADVANCE;
          Serial.print("State: SERVO_VERTICAL_ADVANCE (Row: ");
          Serial.print(currentVerticalRowIndex);
          Serial.println(")");
        } else {
          // All vertical rows are scanned
          currentScanState = SCAN_COMPLETE_RESET;
          Serial.println("State: SCAN_COMPLETE_RESET (Full scan complete)");
        }
      }
      // Lidar readings would be taken continuously while the stepper moves horizontally.
      // E.g., if (stepper.currentPosition() % STEPS_PER_DEGREE_FOR_LIDAR_POINT == 0) { readLidar(); }
      // Or simply: readLidar(); // if lidar reports continuously
      break;

    case SERVO_VERTICAL_ADVANCE:
      if (currentMillis - lastServoMoveMillis >= SERVO_MOVE_SPEED_MS) {
        lastServoMoveMillis = currentMillis;

        int targetServoAngle = SERVO_START_ANGLE + (currentVerticalRowIndex * SERVO_INCREMENT_DEGREES);
        // Ensure target angle doesn't exceed the end or go below start
        if (targetServoAngle > SERVO_END_ANGLE) targetServoAngle = SERVO_END_ANGLE;
        if (targetServoAngle < SERVO_START_ANGLE) targetServoAngle = SERVO_START_ANGLE; // Should not happen with current logic

        myServo.write(targetServoAngle);
        currentServoAngle = targetServoAngle;
        Serial.print("Servo Angle: ");
        Serial.println(currentServoAngle);

        // Once servo reaches its new vertical position, start the next horizontal sweep
        if (currentServoAngle == targetServoAngle) { // Check if servo has reached its commanded position
          currentScanState = STEPPER_HORIZONTAL_SWEEP;
          Serial.println("State: STEPPER_HORIZONTAL_SWEEP");
          // Re-home stepper if desired before next sweep, or let it start from current (zigzag) position
          // stepper.moveTo(0); // If you want to force it back to 0 for each row.
        }
      }
      break;

    case SCAN_COMPLETE_RESET:
      // This state handles returning both motors to their home position after a full scan.
      Serial.println("Full scan complete. Returning to home position...");
      stepper.moveTo(0); // Assuming 0 is the home position for the stepper
      myServo.write(SERVO_START_ANGLE); // Return servo to start angle

      // Wait for stepper to home
      while (stepper.isRunning()) {
        stepper.run(); // Keep running stepper until home
      }
      // Give servo a moment to return
      delay(500);

      Serial.println("Returned to home. Restarting scan cycle.");
      // Reset state variables for a new scan cycle
      currentVerticalRowIndex = 0;
      currentServoAngle = SERVO_START_ANGLE;
      stepperSweepingRight = true; // Always start horizontal scan from left
      currentScanState = STEPPER_HORIZONTAL_SWEEP;
      break;
  }
}

/*
  Important Notes for Tuning and Implementation:

  1.  Stepper Motor Troubleshooting (Crucial!):
      * **Wiring:** Re-verify all connections: motor coils (A1/A2, B1/B2), DRV8825 to Arduino (DIR, STEP, EN, GND), and separate motor power supply. A common ground between Arduino and DRV8825 is essential.
      * **DRV8825 Vref Current:** This is the #1 cause of a non-moving stepper. **YOU MUST SET Vref with a multimeter.** For a 1A motor, set Vref to 0.5V. If Vref is too low, the motor won't have enough torque. Too high, and the DRV8825 will overheat.
      * **ENABLE Pin:** Ensure pin 4 is correctly wired to the EN pin on DRV8825 and `digitalWrite(ENABLE_PIN, LOW);` is correctly enabling it.
      * **Microstepping Jumpers (MS1, MS2, MS3):** For initial debugging, remove all jumpers on the DRV8825 for full-step mode (200 steps/revolution for 1.8-degree motors). Once the stepper moves, you can add jumpers for microstepping (e.g., 1/16th) for smoother motion. **If you use microstepping, you MUST update `STEPS_PER_REVOLUTION` in the code accordingly (e.g., 200 * 16 = 3200 for 1/16 microsteps).**
      * **`setMaxSpeed` and `setAcceleration`:** Start with very conservative values (e.g., `setMaxSpeed(100); setAcceleration(20);`). If the motor moves reliably, slowly increase these values. High values can cause stalling.

  2.  Horizontal Stepper Sweep (Zigzag):
      * `HORIZONTAL_SWEEP_RANGE_STEPS` defines the extent of the horizontal sweep in steps (e.g., 180 degrees worth of steps).
      * `stepperSweepingRight` toggles the direction for each sweep, ensuring the stepper moves back and forth between `0` and `HORIZONTAL_SWEEP_RANGE_STEPS`, preventing wire tangling.
      * The stepper takes its entire 180-degree sweep before the servo advances.

  3.  Vertical Servo Indexing:
      * The servo moves only after a full horizontal sweep by the stepper is complete.
      * `SERVO_INCREMENT_DEGREES` is set to `LIDAR_FOV_DEGREES` (2 degrees), meaning the servo advances precisely by the Lidar's field of view for each new row.
      * `SERVO_MOVE_SPEED_MS` controls how quickly the servo moves to its new position. Adjust this for smooth indexing.

  4.  Lidar Integration:
      * The conceptual Lidar reading would occur *while the stepper is sweeping horizontally*. You would integrate your Lidar's communication (e.g., Serial, I2C, SPI) to acquire data as the stepper moves.
      * For example, inside the `STEPPER_HORIZONTAL_SWEEP` state, you could add:
          `// if (stepper.currentPosition() % (long)(STEPS_PER_REVOLUTION/360 * DESIRED_LIDAR_HORIZONTAL_RESOLUTION) == 0) {`
          `//    readLidarData(); // Call your Lidar reading function here`
          `// }`
          This would trigger a reading every N horizontal steps.

  5.  Scan Completion and Reset:
      * The `SCAN_COMPLETE_RESET` state handles returning both motors to their home (start) positions after the entire scan grid is covered (all vertical rows, each with a horizontal sweep). The scan will then automatically restart.

This revised structure should give you the desired horizontal zigzag sweep with the stepper and vertical indexing with the servo, effectively creating a 2D scan grid with your conceptual Lidar.*/q