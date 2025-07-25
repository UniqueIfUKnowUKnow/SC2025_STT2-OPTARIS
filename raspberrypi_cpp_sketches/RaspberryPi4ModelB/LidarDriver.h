// LidarDriver.h
#pragma once
#include <string>

class LidarDriver {
public:
    LidarDriver(const std::string& name, const std::string& port, int baudRate);
    bool init();
    int getDistance(); // Returns distance in cm, or -1 on error
private:
    std::string lidarName;
    std::string serialPort;
    int baudRate;
    // File descriptor for serial port
    int serial_fd;

    // Helper to read data frame from TFmini-S (specific to its protocol)
    // You'll need to implement the parsing based on TFmini-S datasheet
    // Example: https://www.robotshop.com/media/files/content/benewake/tfmini-s-manual-v1.4.pdf
    // Look for data frame format (0x59 0x59 Dist_L Dist_H Strength_L Strength_H ...)
};
