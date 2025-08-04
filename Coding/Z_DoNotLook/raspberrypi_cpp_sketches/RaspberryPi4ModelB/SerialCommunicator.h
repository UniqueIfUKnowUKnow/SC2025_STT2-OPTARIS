// SerialCommunicator.h
#pragma once
#include <string>
#include <iostream>

class SerialCommunicator {
public:
    SerialCommunicator(const std::string& port, int baudRate);
    ~SerialCommunicator();
    bool openPort();
    void closePort();
    bool writeString(const std::string& data);
    std::string readLine(); // Reads until newline or timeout

private:
    std::string serialPort;
    int baudRate;
    int serial_fd;
};

