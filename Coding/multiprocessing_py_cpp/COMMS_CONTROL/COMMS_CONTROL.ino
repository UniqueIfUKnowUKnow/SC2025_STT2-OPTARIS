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
const int STEPS_PER_REVOLUTION = 6400*3 ; // with full 1/32 microstepping -> 32 steps per 1.8 degrees
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

void makeMovements()
{
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    // do-while loop for upward tilting
    do 
    {
        // Rotating clockwise
        myStepper.step(STEPS_PER_REVOLUTION); // Makes a full revolution clockwise
        //delay(30);
        delay(10);

        while(current_pos < starting_pos + 2)
        {
            // Tilting up
            myServo.write(current_pos);
            //delay(15);
            delay(5);
            current_pos += 1;
        }
        starting_pos += 2;

        // Rotating counter-clockwise
        myStepper.step(-STEPS_PER_REVOLUTION); // Makes a full revolution clockwise
        // delay(30);
        delay(10);

        while(current_pos < starting_pos + 2)
        {
            // Tilting up
            myServo.write(current_pos);
            // delay(15);
            delay(5);
            current_pos += 1;
        }
        starting_pos += 2;
    }
    while(starting_pos < 70);


    // do-while loop for downward tilting
    do 
    {
        // Rotating Clockwise
        myStepper.step(STEPS_PER_REVOLUTION); // Makes a full revolution clockwise
        // delay(30);
        delay(10);

        while(current_pos > starting_pos + 2)
        {
            // Tilting down
            myServo.write(current_pos);
            // delay(15);
            delay(5);
            current_pos -= 1;
        }
        starting_pos -= 2;

        // Rotating counter-clockwise
        myStepper.step(-STEPS_PER_REVOLUTION); // Makes a full revolution clockwise
        // delay(30);
        delay(10);

        while(current_pos > starting_pos + 2)
        {
            // Tilting down
            myServo.write(current_pos);
            // delay(15);
            delay(5);
            current_pos -= 1;
        }
        starting_pos -= 2;
    }
    while(starting_pos > 0);
}

void communicateWithRaspberryPi()
{
    auto last_execution_time = std::chrono::high_resolution_clock::now();
    auto delay = std::chrono::milliseconds(10);

    auto start_time = std::chrono::high_resolution_clock::now();
    while (std::chrono::high_resolution_clock::now() - start_time < std::chrono::seconds(1)) 
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = current_time - last_execution_time;

        if (elapsed_time >= delay) 
        {
            if (Serial.available() > 0) {
                String receivedStringArduino = Serial.readStringUntil('\n');
                receivedStringArduino.trim(); 

                std::string receivedStdString(receivedStringArduino.c_str());

                int received_5_digit_number = std::stoi(receivedStdString);
                
                int number_to_send_arduino_to_pi = 8820700; // Example number to send back to Raspberry Pi

                std::string string_to_send = std::to_string(number_to_send_arduino_to_pi);
                
                char largeNumberCharBuffer[9]; 
                sprintf(largeNumberCharBuffer, "%08d", number_to_send_arduino_to_pi);
                
                String largeNumberString = String(largeNumberCharBuffer);

                Serial.print(largeNumberString);

                dataExtract::ExtractedNumbers extracted_on_arduino = data_extractor.extractDigits(largeNumberString.c_str());
            }
            
            last_execution_time = current_time;
        }
    }
    
    if (Serial.available() > 0) {
        String receivedStringArduino = Serial.readStringUntil('\n');
        receivedStringArduino.trim(); 

        std::string receivedStdString(receivedStringArduino.c_str());

        int received_5_digit_number = std::stoi(receivedStdString);
        
        int number_to_send_arduino_to_pi = 8820700; // Example number to send back to Raspberry Pi

        std::string string_to_send = std::to_string(number_to_send_arduino_to_pi);
        
        char largeNumberCharBuffer[9]; 
        sprintf(largeNumberCharBuffer, "%08d", number_to_send_arduino_to_pi);
        
        String largeNumberString = String(largeNumberCharBuffer);

        Serial.print(largeNumberString);

        dataExtract::ExtractedNumbers extracted_on_arduino = data_extractor.extractDigits(largeNumberString.c_str());
    }
    else continue;
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
    // Create and run the two methods on separate threads
    std::thread movement_thread(makeMovements);
    std::thread timer_thread(communicateWithRaspberryPi);

    // Wait for both threads to finish
    movement_thread.join();
    timer_thread.join();
}