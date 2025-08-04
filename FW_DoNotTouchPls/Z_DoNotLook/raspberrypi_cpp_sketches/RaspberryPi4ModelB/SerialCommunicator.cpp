// SerialCommunicator.cpp (Simplified)
#include "SerialCommunicator.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

SerialCommunicator::SerialCommunicator(const std::string& port, int baudRate)
    : serialPort(port), baudRate(baudRate), serial_fd(-1) {}

SerialCommunicator::~SerialCommunicator() {
    closePort();
}

bool SerialCommunicator::openPort() {
    serial_fd = open(serialPort.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        std::cerr << "Error opening serial port: " << serialPort << std::endl;
        return false;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);

    // Set baud rate
    cfsetispeed(&options, B115200); // Use a fixed common baud rate for Arduino
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE; // Clear data size bits
    options.c_cflag |= CS8; // 8 data bits

    // Raw input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non-canonical input
    options.c_oflag &= ~OPOST; // Raw output

    // VMIN, VTIME
    options.c_cc[VMIN] = 0; // Read at least 0 characters
    options.c_cc[VTIME] = 5; // Wait 0.5 seconds (5 * 0.1s) for data

    tcsetattr(serial_fd, TCSANOW, &options);

    // Clear buffer
    tcflush(serial_fd, TCIOFLUSH);
    return true;
}

void SerialCommunicator::closePort() {
    if (serial_fd != -1) {
        close(serial_fd);
        serial_fd = -1;
    }
}

bool SerialCommunicator::writeString(const std::string& data) {
    if (serial_fd == -1) return false;
    ssize_t bytes_written = write(serial_fd, data.c_str(), data.length());
    return bytes_written == (ssize_t)data.length();
}

std::string SerialCommunicator::readLine() {
    std::string line = "";
    char buffer[256];
    int bytesRead;
    while ((bytesRead = read(serial_fd, buffer, sizeof(buffer) - 1)) > 0) {
        buffer[bytesRead] = '\0'; // Null-terminate the received data
        line += buffer;
        if (line.find('\n') != std::string::npos) {
            break; // Found a newline, return the line
        }
    }
    return line;
}
