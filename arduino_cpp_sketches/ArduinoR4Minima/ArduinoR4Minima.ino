// Arduino_Motor_Control.ino
#include <Servo.h>

// Stepper Motor Pins (DRV8825)
const int STEP_PIN = 2; // Connect to DRV8825 STEP
const int DIR_PIN = 3;  // Connect to DRV8825 DIR
// const int ENABLE_PIN = 4; // Optional: Connect to DRV8825 ENABLE (active low)
const int MS1_PIN = 5; // Microstepping pins
const int MS2_PIN = 6;
const int MS3_PIN = 7;

// Servo Motor Pin
const int SERVO_PIN = 9; // Connect to MG90S Signal

Servo elevationServo; // Servo object

// Stepper motor parameters
const int STEPS_PER_REV = 200; // For FIT0278, check datasheet
const int MICROSTEPS = 8; // Adjust based on MS1, MS2, MS3 settings (e.g., 8 for 1/8 microstepping)
const float STEPS_PER_DEGREE_AZ = (float)STEPS_PER_REV * MICROSTEPS / 360.0; // Steps per degree for azimuth

// Target angles from Raspberry Pi (global to be updated)
volatile float targetAzimuth = 0.0; // Degrees
volatile float targetElevation = 0.0; // Degrees

// Current angles (for motor positioning)
float currentAzimuth = 0.0;
float currentElevation = 0.0;

// Function prototypes
void setStepperMicrostepping(int microsteps);
void moveStepper(float degrees);
void moveServo(float degrees);
void parseCommand(String command);

void setup() {
    Serial.begin(115200); // Must match Raspberry Pi's baud rate

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    // pinMode(ENABLE_PIN, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(MS3_PIN, OUTPUT);

    elevationServo.attach(SERVO_PIN);

    // Initial microstepping (e.g., 1/8 microstepping)
    setStepperMicrostepping(MICROSTEPS);

    // Enable stepper driver
    // digitalWrite(ENABLE_PIN, LOW); // DRV8825 enable is active low

    // Set initial servo position (e.g., center)
    elevationServo.write(90); // Servo typically 0-180 degrees
    currentElevation = 90.0; // Our internal representation of degrees

    // Set initial stepper position (assumed 0)
    currentAzimuth = 0.0;
    
    Serial.println("Arduino Ready.");
}

void loop() {
    // Check for incoming serial commands from Raspberry Pi
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        parseCommand(command);
    }

    // Move motors to target positions if they are not already there
    // This is a simple proportional move; a PID loop could be implemented here too
    // for very fine control, but RPi is handling the main PID.
    
    // Azimuth (Stepper) control
    float az_diff = targetAzimuth - currentAzimuth;
    if (abs(az_diff) > 0.1) { // Move if difference is significant
        moveStepper(az_diff);
        currentAzimuth += az_diff; // Update current position
        currentAzimuth = fmod(currentAzimuth, 360.0); // Keep within 0-360
        if (currentAzimuth < 0) currentAzimuth += 360.0;
    }

    // Elevation (Servo) control
    float el_diff = targetElevation - currentElevation;
    if (abs(el_diff) > 0.5) { // Move if difference is significant (servos less precise than steppers)
        moveServo(targetElevation); // Servo moves directly to angle
        currentElevation = targetElevation;
    }

    delay(10); // Small delay to prevent busy-waiting and allow serial buffer to fill
}

void parseCommand(String command) {
    // Command format: AZ<azimuth_int_x100>EL<elevation_int_x100>
    // Example: AZ1234EL5678 (12.34 degrees Az, 56.78 degrees El)

    int azIndex = command.indexOf("AZ");
    int elIndex = command.indexOf("EL");

    if (azIndex != -1 && elIndex != -1) {
        String azStr = command.substring(azIndex + 2, elIndex);
        String elStr = command.substring(elIndex + 2);

        // Convert to float from int*100
        targetAzimuth = azStr.toFloat() / 100.0;
        targetElevation = elStr.toFloat() / 100.0;

        // Optional: Clamp angles to safe limits for your physical setup
        targetAzimuth = fmod(targetAzimuth, 360.0);
        if (targetAzimuth < 0) targetAzimuth += 360.0;
        targetElevation = constrain(targetElevation, 0.0, 180.0); // MG90S usually 0-180 degrees
        
        Serial.print("Arduino Rx: Az=");
        Serial.print(targetAzimuth);
        Serial.print(", El=");
        Serial.println(targetElevation);
    } else {
        Serial.println("Arduino Rx: Invalid command format.");
    }
}

void setStepperMicrostepping(int microsteps) {
    // DRV8825 Microstepping Table:
    // MS1 MS2 MS3 | Microstep Resolution
    // L   L   L   | Full step
    // H   L   L   | Half step
    // L   H   L   | 1/4 step
    // H   H   L   | 1/8 step
    // L   L   H   | 1/16 step
    // H   L   H   | 1/32 step
    // L   H   H   | 1/32 step
    // H   H   H   | 1/32 step (usually same as previous for some drivers)
    
    // For simplicity, let's just implement 1/8 microstepping (HHL)
    // You can extend this for other resolutions if needed.
    if (microsteps == 8) {
        digitalWrite(MS1_PIN, HIGH);
        digitalWrite(MS2_PIN, HIGH);
        digitalWrite(MS3_PIN, LOW);
    } else { // Default to Full Step if not 1/8
        digitalWrite(MS1_PIN, LOW);
        digitalWrite(MS2_PIN, LOW);
        digitalWrite(MS3_PIN, LOW);
        Serial.println("Warning: Stepper set to Full Step. Configure for desired microstepping.");
    }
}

void moveStepper(float degrees) {
    if (degrees == 0) return;

    int steps = round(degrees * STEPS_PER_DEGREE_AZ);
    
    if (steps > 0) {
        digitalWrite(DIR_PIN, HIGH); // Clockwise
    } else {
        digitalWrite(DIR_PIN, LOW); // Counter-clockwise
        steps = abs(steps);
    }

    for (int i = 0; i < steps; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500); // Adjust this delay for stepper speed
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500); // Adjust this delay for stepper speed
    }
}

void moveServo(float degrees) {
    // Map the incoming degrees to the servo's 0-180 range if your system's elevation
    // isn't directly 0-180 (e.g., if -30 to +90 degrees maps to 0-180 servo pulses)
    // For now, assuming direct mapping for simplicity
    elevationServo.write(static_cast<int>(degrees));
}