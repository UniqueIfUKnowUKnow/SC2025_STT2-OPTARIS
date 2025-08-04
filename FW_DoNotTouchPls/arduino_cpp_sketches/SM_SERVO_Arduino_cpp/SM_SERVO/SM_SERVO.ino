//Stepper Motor definition
//==================================================================================================================================================================
//==================================================================================================================================================================
#include <Stepper.h>

const int DIR_PIN = 2;     // Digital pin connected to DRV8825 DIR pin
const int STEP_PIN = 3;    // Digital pin connected to DRV8825 STEP pin
const int ENABLE_PIN = 4;  // Digital pin connected to DRV8825 EN pin (LOW to enable, HIGH to disable)

// Define the number of steps per revolution for the motor
// Most common NEMA 17 motors are 50 steps/revolution for full step mode// If the motor is 1.8 degrees per step, then 360 / 1.8 = 50 steps
const int STEPS_PER_REVOLUTION = 6400 ; // with full 1/32 microstepping -> 32 steps per 1.8 degrees
                                          // Therefore for 90 degrees -> 1600 steps

Stepper myStepper(STEPS_PER_REVOLUTION, STEP_PIN, DIR_PIN);
//==================================================================================================================================================================
//==================================================================================================================================================================



//Servo definition
//==================================================================================================================================================================
//==================================================================================================================================================================
#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 9;

//int pos = 0;

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
  Serial.println("Stepper Motor Control Start (using Stepper.h)");

  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);
  Serial.println("Driver Enabled.");

  myStepper.setSpeed(90); // 60 RPM (1 revolution per second)
}



void loop()
{
  // do-while loop for clockwise rotations
  do 
  {
    Serial.println("Rotating Clockwise...");
    myStepper.step(STEPS_PER_REVOLUTION); // Makes a full revolution clockwise
    Serial.println("Clockwise rotation complete.");
    //delay(30);
    delay(10);

    while(current_pos < starting_pos + 2)
    {
      myServo.write(current_pos);
      //delay(15);
      delay(5);
      current_pos += 1;
    }
    starting_pos += 2;

    Serial.println("Rotating Clockwise...");
    myStepper.step(-STEPS_PER_REVOLUTION); // Makes a full revolution clockwise
    Serial.println("Clockwise rotation complete.");
    // delay(30);
    delay(10);

    while(current_pos < starting_pos + 2)
    {
      myServo.write(current_pos);
      // delay(15);
      delay(5);
      current_pos += 1;
    }
    starting_pos += 2;
  }
  while(starting_pos < 70);

  // do-while loop for counter-clockwise rotations
  do 
  {
    Serial.println("Rotating Clockwise...");
    myStepper.step(STEPS_PER_REVOLUTION); // Makes a full revolution clockwise
    Serial.println("Clockwise rotation complete.");
    // delay(30);
    delay(10);

    while(current_pos > starting_pos + 2)
    {
      myServo.write(current_pos);
      // delay(15);
      delay(5);
      current_pos -= 1;
    }
    starting_pos -= 2;

    Serial.println("Rotating Clockwise...");
    myStepper.step(-STEPS_PER_REVOLUTION); // Makes a full revolution clockwise
    Serial.println("Clockwise rotation complete.");
    // delay(30);
    delay(10);

    while(current_pos > starting_pos + 2)
    {
      myServo.write(current_pos);
      // delay(15);
      delay(5);
      current_pos -= 1;
    }
    starting_pos -= 2;
  }
  while(starting_pos > 0);
}


// void loop()
// {
//   Serial.println("Rotating Clockwise...");
//   myStepper.step(STEPS_PER_REVOLUTION); // Makes a full revolution clockwise
//   Serial.println("Clockwise rotation complete.");
//   delay(30);

//   for(pos = 0; pos <= 2; pos += 1) // pos <= degrees of movement
//   {
//     myServo.write(pos);
//     delay(15);
//   }

//   Serial.println("Rotating Counter-Clockwise...");
//   myStepper.step(-STEPS_PER_REVOLUTION); // Makes a full revolution counter-clockwise
//   Serial.println("Counter-clockwise rotation complete.");
//   delay(500);

//   for (pos = 2; pos >= 0; pos -= 1) // pos = degrees of movement
//   {
//     myServo.write(pos);
//     delay(15);
//   }

  
// }

