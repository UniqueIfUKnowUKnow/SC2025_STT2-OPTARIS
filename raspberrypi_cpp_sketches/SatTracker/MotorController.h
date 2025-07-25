#pragma once
#include <cstdint>

struct MotorPosition {
    double pan_angle;      // Pan angle in radians
    double tilt_angle;     // Tilt angle in radians
    bool pan_moving;       // Pan axis motion status
    bool tilt_moving;      // Tilt axis motion status
};

struct MotorLimits {
    double pan_min, pan_max;     // Pan limits in radians
    double tilt_min, tilt_max;   // Tilt limits in radians
    double max_pan_velocity;     // Maximum pan velocity (rad/s)
    double max_tilt_velocity;    // Maximum tilt velocity (rad/s)
};

class MotorController {
private:
    uint8_t i2c_address;
    int i2c_handle;
    MotorPosition current_position;
    MotorPosition target_position;
    MotorLimits limits;

    // PID controllers for each axis
    struct PIDController {
        double kp, ki, kd;           // PID gains
        double integral;             // Integral accumulator
        double last_error;           // Previous error for derivative
        double output_limit;         // Output saturation limit
    } pan_pid, tilt_pid;

    // Communication with Arduino
    bool sendI2CCommand(uint8_t command, const uint8_t* data, size_t length);
    bool receiveI2CData(uint8_t* data, size_t length);
    void updatePIDController(PIDController& pid, double error, double dt);

    // Safety and limits
    bool checkLimits(double pan, double tilt);
    void enforceVelocityLimits(double& pan_vel, double& tilt_vel);

public:
    MotorController(uint8_t i2c_addr);
    ~MotorController();

    bool initialize();
    void calibrate();
    void setPosition(double pan_angle, double tilt_angle);
    void setVelocity(double pan_velocity, double tilt_velocity);
    MotorPosition getCurrentPosition() const;
    void setPIDGains(double pan_kp, double pan_ki, double pan_kd,
                     double tilt_kp, double tilt_ki, double tilt_kd);
    void setLimits(const MotorLimits& motor_limits);
    void emergencyStop();
    bool isCalibrated() const;
    void update(double delta_time);
};
