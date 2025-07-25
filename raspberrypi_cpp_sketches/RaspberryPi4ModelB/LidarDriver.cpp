// LidarDriver.cpp (Simplified)
#include "LidarDriver.h"
#include <iostream>
#include <fcntl.h> // File control options
#include <unistd.h> // UNIX standard functions
#include <termios.h> // POSIX terminal control definitions
#include <array>

LidarDriver::LidarDriver(const std::string& name, const std::string& port, int baudRate)
    : lidarName(name), serialPort(port), baudRate(baudRate), serial_fd(-1) {}

bool LidarDriver::init() {
    serial_fd = open(serialPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        std::cerr << "Error opening LiDAR serial port: " << serialPort << std::endl;
        return false;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B115200); // TFmini-S default baud rate
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE; // Clear data size bits
    options.c_cflag |= CS8; // 8 data bits
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_oflag &= ~OPOST; // Raw output
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable flow control
    options.c_cc[VMIN] = 1; // Read at least 1 byte
    options.c_cc[VTIME] = 0; // No timeout (or set a small timeout)

    tcsetattr(serial_fd, TCSANOW, &options);
    std::cout << lidarName << " initialized on " << serialPort << std::endl;
    return true;
}

int LidarDriver::getDistance() {
    std::array<unsigned char, 9> buffer; // TFmini-S frame is 9 bytes
    int bytesRead = read(serial_fd, buffer.data(), buffer.size());

    if (bytesRead >= 9) {
        // Find header 0x59 0x59
        int header_pos = -1;
        for (int i = 0; i < bytesRead - 1; ++i) {
            if (buffer[i] == 0x59 && buffer[i+1] == 0x59) {
                header_pos = i;
                break;
            }
        }

        if (header_pos != -1 && (bytesRead - header_pos) >= 9) {
            unsigned char* frame = buffer.data() + header_pos;
            unsigned short dist = frame[2] | (frame[3] << 8); // Distance in cm
            unsigned short strength = frame[4] | (frame[5] << 8);
            // Checksum verification (optional but good practice)
            unsigned char checksum = 0;
            for (int i = 0; i < 8; ++i) {
                checksum += frame[i];
            }
            if (checksum == frame[8]) {
                return dist;
            } else {
                // std::cerr << "LiDAR Checksum error." << std::endl;
            }
        }
    }
    // No valid frame or not enough bytes
    return -1;
}

// Don't forget to implement close in destructor or explicitly.
// LidarDriver::~LidarDriver() { if (serial_fd != -1) close(serial_fd); }
