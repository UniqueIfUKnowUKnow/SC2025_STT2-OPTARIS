#include <Stepper.h>

const int DIR_PIN = 2;     // Digital pin connected to DRV8825 DIR pin
const int STEP_PIN = 3;    // Digital pin connected to DRV8825 STEP pin
const int ENABLE_PIN = 4;  // Digital pin connected to DRV8825 EN pin (LOW to enable, HIGH to disable)

// Define the number of steps per revolution for the motor
// Most common NEMA 17 motors are 200 steps/revolution for full step mode
// If the motor is 1.8 degrees per step, then 360 / 1.8 = 200 steps
const int STEPS_PER_REVOLUTION = 600;

Stepper myStepper(STEPS_PER_REVOLUTION, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper Motor Control Start (using Stepper.h)");

  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);
  Serial.println("Driver Enabled.");

  myStepper.setSpeed(120); // 60 RPM (1 revolution per second)
}

void loop()
{
  Serial.println("Rotating Clockwise...");
  myStepper.step(STEPS_PER_REVOLUTION); // Makes a full revolution clockwise
  Serial.println("Clockwise rotation complete.");
  delay(1000);

  Serial.println("Rotating Counter-Clockwise...");
  myStepper.step(-STEPS_PER_REVOLUTION); // Makes a full revolution counter-clockwise
  Serial.println("Counter-clockwise rotation complete.");
  delay(2000);
}