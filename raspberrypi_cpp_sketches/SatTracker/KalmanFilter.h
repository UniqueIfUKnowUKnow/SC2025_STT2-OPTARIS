#pragma once
#include <vector>
#include <array>

class KalmanFilter {
private:
    static constexpr int STATE_SIZE = 9;  // [x, y, z, vx, vy, vz, ax, ay, az]
    static constexpr int MEASUREMENT_SIZE = 3;  // [range, azimuth, elevation]

    std::array<double, STATE_SIZE> state;                    // State vector
    std::array<std::array<double, STATE_SIZE>, STATE_SIZE> P; // Error covariance
    std::array<std::array<double, STATE_SIZE>, STATE_SIZE> Q; // Process noise
    std::array<std::array<double, MEASUREMENT_SIZE>, MEASUREMENT_SIZE> R; // Measurement noise

    // Kalman filter matrices
    void predictState(double dt);
    void updateState(const std::array<double, MEASUREMENT_SIZE>& measurement);
    void computeJacobians(double dt);
    std::array<double, MEASUREMENT_SIZE> measurementModel(const std::array<double, STATE_SIZE>& state);

public:
    KalmanFilter();
    ~KalmanFilter() = default;

    void initialize(double x, double y, double z);
    void predict(double delta_time);
    void update(double range, double azimuth, double elevation);

    std::array<double, 3> getPosition() const;
    std::array<double, 3> getVelocity() const;
    std::array<double, 3> getAcceleration() const;
    double getPositionUncertainty() const;

    void setProcessNoise(double position_noise, double velocity_noise, double acceleration_noise);
    void setMeasurementNoise(double range_noise, double angle_noise);
};
