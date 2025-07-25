#pragma once
#include <string>
#include <vector>
#include <mutex>

struct LidarReading {
    double distance;        // Range in meters
    double angle;          // Bearing angle in radians
    double timestamp;      // System timestamp
    uint8_t strength;      // Signal strength
    bool valid;           // Data validity flag
};

class SensorManager {
private:
    int serial_fd;
    std::string device_path;
    int baud_rate;
    std::vector<uint8_t> rx_buffer;
    LidarReading current_reading;
    std::mutex data_mutex;

    // LiDAR protocol constants
    static constexpr uint8_t FRAME_HEADER = 0x59;
    static constexpr size_t FRAME_LENGTH = 9;

    bool parseFrame(const std::vector<uint8_t>& frame);
    uint16_t calculateChecksum(const std::vector<uint8_t>& data);
    void processSerialData();

public:
    SensorManager(const std::string& device, int baud);
    ~SensorManager();

    bool initialize();
    void calibrate();
    void update();
    LidarReading getCurrentReading();
    bool isConnected() const;
    void setMeasurementMode(uint8_t mode);
};
