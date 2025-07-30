// Include the AccelStepper library for easier stepper motor control.
// You might need to install this library via Arduino IDE:
// Sketch > Include Library > Manage Libraries...
// Search for "AccelStepper" and install it.
#include <AccelStepper.h>

// Define the pins connected to the DRV8825 driver
const int DIR_PIN = 2;   // Digital pin connected to DRV8825 DIR pin
const int STEP_PIN = 3;  // Digital pin connected to DRV8825 STEP pin
const int ENABLE_PIN = 4; // Digital pin connected to DRV8825 EN pin (LOW to enable, HIGH to disable)

// Define the number of steps per revolution for your motor.
// Most common NEMA 17 motors are 200 steps/revolution for full step mode.
// If your motor is 1.8 degrees per step, then 360 / 1.8 = 200 steps.
const int STEPS_PER_REVOLUTION = 200;

// Create an instance of the AccelStepper class.
// The first parameter (1) indicates that we are using the driver in STEP/DIR mode.
// The second parameter is the STEP pin, and the third is the DIR pin.
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(9600);
  Serial.println("Stepper Motor Control Start");

  // Set the ENABLE pin as an output
  pinMode(ENABLE_PIN, OUTPUT);

  // Enable the stepper driver (LOW to enable)
  digitalWrite(ENABLE_PIN, LOW);
  Serial.println("Driver Enabled.");

  // Set the maximum speed (steps per second)
  // Adjust this value based on your motor and power supply.
  // Too high a speed can cause the motor to stall or skip steps.
  stepper.setMaxSpeed(1000); // steps/second

  // Set the acceleration (steps per second per second)
  // This helps prevent sudden starts/stops and reduces motor stalling.
  stepper.setAcceleration(500); // steps/second^2
}

void loop() {
  // --- Rotate Clockwise ---
  Serial.println("Rotating Clockwise...");
  // Set the target position to one full revolution clockwise
  stepper.moveTo(STEPS_PER_REVOLUTION);
  // Run the motor until it reaches the target position
  while (stepper.currentPosition() != STEPS_PER_REVOLUTION) {
    stepper.run();
  }
  Serial.println("Clockwise rotation complete.");
  delay(1000); // Wait for 1 second

  // --- Rotate Counter-Clockwise ---
  Serial.println("Rotating Counter-Clockwise...");
  // Set the target position to zero (back to original position)
  stepper.moveTo(0);
  // Run the motor until it reaches the target position
  while (stepper.currentPosition() != 0) {
    stepper.run();
  }
  Serial.println("Counter-clockwise rotation complete.");
  delay(2000); // Wait for 2 seconds before repeating
}

/*
   Important Notes:

   1.  DRV8825 Current Setting: Before powering up the motor, very carefully adjust the
       potentiometer on the DRV8825 to set the Vref voltage. The motor current (I_trip)
       is typically calculated as Vref * 2. So, if your motor is rated for 1A, set Vref to 0.5V.
       Use a small screwdriver and a multimeter to measure Vref (between the potentiometer
       and GND). Setting the current too high can damage the motor or driver.

   2.  Microstepping: This code uses the AccelStepper library, which handles the STEP/DIR
       signals. By default, if MS1, MS2, MS3 pins on the DRV8825 are left floating or
       connected to GND, it operates in full-step mode (200 steps/revolution for a 1.8-degree motor).
       If you want microstepping (e.g., 1/32 microstep), you would need to connect MS1, MS2, MS3
       to appropriate logic levels (HIGH/LOW) as per the DRV8825 datasheet.
       For example, for 1/32 microstep, all MS pins are HIGH. If you change microstepping,
       you must adjust STEPS_PER_REVOLUTION accordingly (e.g., 200 * 32 = 6400 steps for 1 revolution).

   3.  Power Supply: Ensure your external motor power supply can provide enough current
       for your motor. A common 12V supply is usually sufficient for NEMA 17 motors.

   4.  Troubleshooting:
       - Motor not moving, just vibrating: Check motor coil connections (A1/A2, B1/B2) - try swapping one pair.
       - Motor moving erratically or skipping steps: Reduce stepper.setMaxSpeed() or increase stepper.setAcceleration().
         Also, check your motor's current setting on the DRV8825.
       - Driver getting very hot: Reduce the motor current via the potentiometer. Ensure heatsink is attached.
       - Nothing happens: Double-check all wiring, especially power and ground connections. Ensure the ENABLE_PIN is LOW.
*/








// Include the Servo library. This library simplifies controlling servo motors.
// You can find this library in the Arduino IDE's Library Manager if it's not already installed.
#include <Servo.h>

// Create a Servo object. This object will control our servo motor.
Servo myServo;

// Define the digital pin to which the servo's signal wire is connected.
// We are using Digital Pin 9, which is a PWM-capable pin on the Arduino UNO R4 Minima.
const int servoPin = 9;

// Variable to store the current position of the servo.
int pos = 0;

void setup() {
  // Attach the Servo object to the specified pin.
  // This tells the Arduino which pin will send the control signals to the servo.
  myServo.attach(servoPin);

  // Initialize the servo to 0 degrees when the program starts.
  myServo.write(0);
  // Give the servo a moment to reach the initial position.
  delay(1000);
}

void loop() {
  // Sweep the servo from 0 degrees to 180 degrees.
  for (pos = 0; pos <= 180; pos += 1) { // Goes from 0 degrees to 180 degrees
    // In steps of 1 degree
    myServo.write(pos); // Tell servo to go to position in variable 'pos'
    delay(15);          // Waits 15ms for the servo to reach the position
  }

  // Sweep the servo from 180 degrees to 0 degrees.
  for (pos = 180; pos >= 0; pos -= 1) { // Goes from 180 degrees to 0 degrees
    // In steps of 1 degree
    myServo.write(pos); // Tell servo to go to position in variable 'pos'
    delay(15);          // Waits 15ms for the servo to reach the position
  }
}
