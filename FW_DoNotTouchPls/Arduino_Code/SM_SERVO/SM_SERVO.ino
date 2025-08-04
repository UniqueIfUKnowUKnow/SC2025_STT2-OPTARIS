//==================================================================================================================================================================
// Refactored Stepper Motor definition using AccelStepper library
//==================================================================================================================================================================
#include <AccelStepper.h>
#include <Servo.h>

// Stepper Motor Pin Definitions
const int DIR_PIN = 2;       // Digital pin connected to DRV8825 DIR pin
const int STEP_PIN = 3;      // Digital pin connected to DRV8825 STEP pin
const int ENABLE_PIN = 4;    // Digital pin connected to DRV8825 EN pin (LOW to enable, HIGH to disable)

// Define the number of steps per revolution for the motor
// For a NEMA 17 motor at 1.8 degrees/step and 1/32 microstepping:
// (360 / 1.8) steps/rev * 32 microsteps/step = 6400 steps/rev
const int STEPS_PER_REVOLUTION = 6400;

// AccelStepper setup
// The DRV8825 driver requires a STEP and DIR pin, which corresponds to the DRIVER type.
AccelStepper myStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
//==================================================================================================================================================================
//==================================================================================================================================================================


//==================================================================================================================================================================
// Servo definition (unchanged)
//==================================================================================================================================================================
Servo myServo;
const int SERVO_PIN = 9;
int starting_pos = 0;
int current_pos = 0;
//==================================================================================================================================================================
//==================================================================================================================================================================


void setup() 
{
  // Servo calibration
  myServo.attach(SERVO_PIN);
  myServo.write(0);
  delay(30);

  // Stepper Motor calibration
  Serial.begin(9600);
  Serial.println("Stepper Motor Control Start (using AccelStepper.h)");

  // AccelStepper requires setting a maximum speed and acceleration.
  // The original code used a speed of 90 RPM with Stepper.h.
  // To convert to AccelStepper's steps/second:
  // 90 RPM * (6400 steps/rev / 60 sec/min) = 9600 steps/sec
  myStepper.setMaxSpeed(9600.0);
  myStepper.setAcceleration(4000.0); // A starting acceleration value (adjust as needed)

  // Set the enable pin for the driver.
  myStepper.setEnablePin(ENABLE_PIN);
  // Enable the motor driver (by setting the pin LOW).
  myStepper.enableOutputs();
  Serial.println("Driver Enabled.");
}


void loop()
{
  // do-while loop for upward Servo tilt
  do 
  {
    Serial.println("Rotating Clockwise...");
    // Use myStepper.move() to set a relative target position.
    // The `myStepper.run()` call inside the while loop will block
    // until the movement is complete, replicating the original behavior.
    myStepper.move(STEPS_PER_REVOLUTION);
    while (myStepper.distanceToGo() != 0) {
      myStepper.run();
    }
    Serial.println("Clockwise rotation complete.");
    delay(10);

    while(current_pos < starting_pos + 2)
    {
      myServo.write(current_pos);
      delay(5);
      current_pos += 1;
    }
    starting_pos += 2;

    Serial.println("Rotating Counter-Clockwise...");
    myStepper.move(-STEPS_PER_REVOLUTION);
    while (myStepper.distanceToGo() != 0) {
      myStepper.run();
    }
    Serial.println("Counter-clockwise rotation complete.");
    delay(10);

    while(current_pos < starting_pos + 2)
    {
      myServo.write(current_pos);
      delay(5);
      current_pos += 1;
    }
    starting_pos += 2;
  }
  while(starting_pos < 70);


  // do-while loop for downward Servo tilt
  do 
  {
    Serial.println("Rotating Clockwise...");
    myStepper.move(STEPS_PER_REVOLUTION);
    while (myStepper.distanceToGo() != 0) {
      myStepper.run();
    }
    Serial.println("Clockwise rotation complete.");
    delay(10);

    while(current_pos > starting_pos + 2)
    {
      myServo.write(current_pos);
      delay(5);
      current_pos -= 1;
    }
    starting_pos -= 2;

    Serial.println("Rotating Counter-Clockwise...");
    myStepper.move(-STEPS_PER_REVOLUTION);
    while (myStepper.distanceToGo() != 0) {
      myStepper.run();
    }
    Serial.println("Counter-clockwise rotation complete.");
    delay(10);

    while(current_pos > starting_pos + 2)
    {
      myServo.write(current_pos);
      delay(5);
      current_pos -= 1;
    }
    starting_pos -= 2;
  }
  while(starting_pos > 0);
}
