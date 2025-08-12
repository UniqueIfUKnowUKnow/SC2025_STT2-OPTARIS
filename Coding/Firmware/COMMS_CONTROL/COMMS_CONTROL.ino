#ifndef __cplusplus
#error A C++ compiler is required!
#endif

// Necessary Arduino libraries:
#include <Arduino.h>

// Serial Communication and Data Extraction libraries:
#include <string>
#include <vector>
#include <cstdio>
// BAUD_RATE definition for serial communication
#define BAUD_RATE 115200

// Necessary CPP libraries to communicate and control the movements simultaneously:
#undef abs
#include <chrono>
#include <thread>


//Stepper Motor definition
//==================================================================================================================================================================
//==================================================================================================================================================================
#include <Stepper.h>

const int DIR_PIN = 2;     // Digital pin connected to DRV8825 DIR pin
const int STEP_PIN = 3;    // Digital pin connected to DRV8825 STEP pin
const int ENABLE_PIN = 4;  // Digital pin connected to DRV8825 EN pin (LOW to enable, HIGH to disable)

// Define the number of steps per revolution for the motor
// Most common NEMA 17 motors are 50 steps/revolution for full step mode// If the motor is 1.8 degrees per step, then 360 / 1.8 = 50 steps
const int STEPS_PER_REVOLUTION = (6400*1.5)-400; // with full 1/32 microstepping -> 32 steps per 1.8 degrees
                                          // Therefore for 90 degrees -> 1600 steps
                                          // actually 9200 with all 3 Microstepping pins connected to HIGH makes 360 degree turn -> 1/46

Stepper myStepper(STEPS_PER_REVOLUTION, STEP_PIN, DIR_PIN);
//==================================================================================================================================================================
//==================================================================================================================================================================



//Servo definition
//==================================================================================================================================================================
//==================================================================================================================================================================
#include <Servo.h>

Servo myServo;

const int SERVO_PIN = 9;

int starting_pos = 0;
int current_pos = 0;
//==================================================================================================================================================================
//==================================================================================================================================================================


class dataExtract {
public:
    struct ExtractedNumbers {
        int motorAngle;
        int servoAngle;
    };

    ExtractedNumbers extractDigits(const std::string& receivedValueStr) {
        ExtractedNumbers result;

        if (receivedValueStr.length() != 8) {
            result.motorAngle = -1;
            result.servoAngle = -1;
            return result;
        }

        result.motorAngle = std::stoi(receivedValueStr.substr(0, 4));
        result.servoAngle = std::stoi(receivedValueStr.substr(4, 4));
        return result;
    }
};

dataExtract data_extractor;


// Non-blocking states and variables for movements
enum MovementState {
    MOVING_UP_CW,
    MOVING_UP_CCW,
    MOVING_DOWN_CW,
    MOVING_DOWN_CCW,
    STOPPED
};

MovementState currentMovementState = STOPPED;

unsigned long lastStepTime = 0;
unsigned long stepInterval = 10; // Stepper motor step delay in milliseconds

void nonBlockingMovements() {
    if (currentMovementState == STOPPED) {
        if (starting_pos < 70) {
            currentMovementState = MOVING_UP_CW;
        } else if (starting_pos >= 70 && current_pos > 0) {
            currentMovementState = MOVING_DOWN_CW;
        }
    }

    if (millis() - lastStepTime < stepInterval) {
        return; // Wait for the next step interval
    }
    lastStepTime = millis();

    switch (currentMovementState) {
        case MOVING_UP_CW:
            myStepper.step(STEPS_PER_REVOLUTION); // Rotate clockwise
            if (current_pos < starting_pos + 2) {
                myServo.write(current_pos);
                current_pos++;
            } else {
                starting_pos += 2;
                currentMovementState = MOVING_UP_CCW;
            }
            break;

        case MOVING_UP_CCW:
            myStepper.step(-STEPS_PER_REVOLUTION); // Rotate counter-clockwise
            if (current_pos < starting_pos + 2) {
                myServo.write(current_pos);
                current_pos++;
            } else {
                starting_pos += 2;
                if (starting_pos >= 70) {
                    currentMovementState = STOPPED;
                } else {
                    currentMovementState = MOVING_UP_CW;
                }
            }
            break;

        case MOVING_DOWN_CW:
            myStepper.step(STEPS_PER_REVOLUTION); // Rotate clockwise
            if (current_pos > starting_pos + 2) {
                myServo.write(current_pos);
                current_pos--;
            } else {
                starting_pos -= 2;
                currentMovementState = MOVING_DOWN_CCW;
            }
            break;

        case MOVING_DOWN_CCW:
            myStepper.step(-STEPS_PER_REVOLUTION); // Rotate counter-clockwise
            if (current_pos > starting_pos + 2) {
                myServo.write(current_pos);
                current_pos--;
            } else {
                starting_pos -= 2;
                if (starting_pos <= 0) {
                    currentMovementState = STOPPED;
                } else {
                    currentMovementState = MOVING_DOWN_CW;
                }
            }
            break;

        case STOPPED:
            // Do nothing or check for a new command
            break;
    }
}


unsigned long lastSerialTime = 0;
unsigned long serialInterval = 10; // Check for serial data every 10 milliseconds

void communicateWithRaspberryPi()
{
    if (Serial.available() > 0) {
        String receivedStringArduino = Serial.readStringUntil('\n');
        receivedStringArduino.trim();

        std::string receivedStdString(receivedStringArduino.c_str());
        
        int number_to_send_arduino_to_pi = 8820700; // Example number to send back to Raspberry Pi

        char largeNumberCharBuffer[9];
        sprintf(largeNumberCharBuffer, "%08d", number_to_send_arduino_to_pi);
        
        String largeNumberString = String(largeNumberCharBuffer);

        Serial.print(largeNumberString);

        dataExtract::ExtractedNumbers extracted_on_arduino = data_extractor.extractDigits(largeNumberString.c_str());
    }
}

void setup() {
    // Servo calibration
    myServo.attach(SERVO_PIN);
    myServo.write(0);
    delay(30);

    // Stepper Motor calibration
    Serial.begin(BAUD_RATE);

    pinMode(ENABLE_PIN, OUTPUT);

    digitalWrite(ENABLE_PIN, LOW);

    myStepper.setSpeed(120); // 60 RPM (1 revolution per second)
}

void loop() {
    nonBlockingMovements();
    
    if (millis() - lastSerialTime >= serialInterval) {
        communicateWithRaspberryPi();
        lastSerialTime = millis();
    }
}