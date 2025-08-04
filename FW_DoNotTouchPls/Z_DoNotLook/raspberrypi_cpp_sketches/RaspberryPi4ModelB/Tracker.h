// Tracker.h
#pragma once
#include <string>
#include "SerialCommunicator.h" // Include SerialCommunicator
#include <map>

class Tracker {
public:
    Tracker();
    void sendCommandToArduino(SerialCommunicator& serial, double azimuth, double elevation);
    double calculatePID(double error, const std::string& axis); // "azimuth" or "elevation"

private:
    // PID coefficients (tune these!)
    // For Azimuth
    double Kp_az = 0.5; // Proportional gain
    double Ki_az = 0.01; // Integral gain
    double Kd_az = 0.1; // Derivative gain
    double prevError_az = 0;
    double integral_az = 0;

    // For Elevation
    double Kp_el = 0.5;
    double Ki_el = 0.01;
    double Kd_el = 0.1;
    double prevError_el = 0;
    double integral_el = 0;

    double maxIntegral = 10.0; // Anti-windup for integral term
    double maxOutput = 5.0; // Max adjustment in degrees per step

};

