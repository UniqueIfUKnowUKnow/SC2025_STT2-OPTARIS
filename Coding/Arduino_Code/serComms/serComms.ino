#ifndef __cplusplus
#error A C++ compiler is required!
#endif

#include <Arduino.h>
#include <string>
#include <vector>
#include <cstdio>

#define BAUD_RATE 115200

class dataExtract {
public:
    struct ExtractedNumbers {
        int motorAngle;
        int servoAngle;
    };

    ExtractedNumbers extractDigits(const std::string& receivedValueStr) {
        ExtractedNumbers result;

        if (receivedValueStr.length() != 8) {
            //Serial.print("Error: Input string must be exactly 9 digits long. Length: ");
            //Serial.println(receivedValueStr.length());
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

void setup() {
    Serial.begin(BAUD_RATE);
    //Serial.println("Arduino: Serial communication started.");
    //Serial.println("Arduino: Waiting for data from Raspberry Pi...");
}

void loop() {
    if (Serial.available() > 0) {
        String receivedStringArduino = Serial.readStringUntil('\n');
        receivedStringArduino.trim(); 

        //Serial.print("\n[Arduino <- RPi] Received 5-digit string: ");
        //Serial.println(receivedStringArduino);

        std::string receivedStdString(receivedStringArduino.c_str());

        int received_5_digit_number = std::stoi(receivedStdString);
        //Serial.print("[Arduino <- RPi] Converted to int: ");
        //Serial.println(received_5_digit_number);
        
        int number_to_send_arduino_to_pi = 8820700; // Example number to send back to Raspberry Pi

        std::string string_to_send = std::to_string(number_to_send_arduino_to_pi);
        
        char largeNumberCharBuffer[9]; 
        sprintf(largeNumberCharBuffer, "%08d", number_to_send_arduino_to_pi);
        
        String largeNumberString = String(largeNumberCharBuffer);

        //Serial.print("[Arduino -> RPi] Sending 8-digit string: ");
        //Serial.println(largeNumberString);

        //Serial.println(largeNumberString);
        Serial.print(largeNumberString);

        //Serial.println("[Arduino: Demonstrating internal digit extraction]");
        dataExtract::ExtractedNumbers extracted_on_arduino = data_extractor.extractDigits(largeNumberString.c_str());

        //Serial.print("  Extracted motorAngle:    "); Serial.println(extracted_on_arduino.motorAngle);
        //Serial.print("  Extracted servoAngle:    "); Serial.println(extracted_on_arduino.servoAngle);
    }
}