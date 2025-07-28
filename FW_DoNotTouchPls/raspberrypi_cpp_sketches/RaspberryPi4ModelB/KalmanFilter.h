// KalmanFilter.h
#pragma once
#include <vector>
#include <Eigen/Dense> // Requires Eigen library for matrix operations

class KalmanFilter {
public:
    // State: [azimuth, elevation, azimuth_velocity, elevation_velocity]
    // Measurement: [measured_azimuth, measured_elevation]
    KalmanFilter();
    void initializeState(double initial_az, double initial_el, double initial_az_vel, double initial_el_vel);
    void predict();
    void update(const std::vector<double>& measurement);
    std::vector<double> getState() const;

private:
    Eigen::VectorXd x_hat; // State estimate vector
    Eigen::MatrixXd P;     // Covariance matrix
    Eigen::MatrixXd F;     // State transition matrix
    Eigen::MatrixXd H;     // Measurement matrix
    Eigen::MatrixXd Q;     // Process noise covariance
    Eigen::MatrixXd R;     // Measurement noise covariance
    Eigen::MatrixXd I;     // Identity matrix

    double dt; // Time step
};

