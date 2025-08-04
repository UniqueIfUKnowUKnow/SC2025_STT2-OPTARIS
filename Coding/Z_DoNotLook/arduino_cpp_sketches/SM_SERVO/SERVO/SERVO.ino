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
