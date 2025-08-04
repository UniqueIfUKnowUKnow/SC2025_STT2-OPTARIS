// Tracker.cpp (Simplified)
#include "Tracker.h"
#include <iostream>
#include <cmath> // For std::abs

Tracker::Tracker() {}

void Tracker::sendCommandToArduino(SerialCommunicator& serial, double azimuth, double elevation) {
    // Format: "AZ<azimuth_degrees>EL<elevation_degrees>\n"
    std::string command = "AZ" + std::to_string(static_cast<int>(azimuth * 100)) + "EL" + std::to_string(static_cast<int>(elevation * 100)) + "\n";
    serial.writeString(command);
    // std::cout << "Sent to Arduino: " << command; // For debugging
}

double Tracker::calculatePID(double error, const std::string& axis) {
    double Kp, Ki, Kd;
    double& prevError = (axis == "azimuth") ? prevError_az : prevError_el;
    double& integral = (axis == "azimuth") ? integral_az : integral_el;

    if (axis == "azimuth") {
        Kp = Kp_az; Ki = Ki_az; Kd = Kd_az;
    } else { // "elevation"
        Kp = Kp_el; Ki = Ki_el; Kd = Kd_el;
    }

    // Proportional term
    double p_term = Kp * error;

    // Integral term (with anti-windup)
    integral += error;
    if (integral > maxIntegral) integral = maxIntegral;
    if (integral < -maxIntegral) integral = -maxIntegral;
    double i_term = Ki * integral;

    // Derivative term
    double derivative = error - prevError;
    double d_term = Kd * derivative;

    // Update previous error
    prevError = error;

    double output = p_term + i_term + d_term;

    // Clamp output
    if (output > maxOutput) output = maxOutput;
    if (output < -maxOutput) output = -maxOutput;

    return output;
}
