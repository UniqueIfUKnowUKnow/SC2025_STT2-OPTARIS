// KalmanFilter.cpp (Conceptual, requires full Eigen setup and math)
#include "KalmanFilter.h"
#include <iostream>

KalmanFilter::KalmanFilter() : dt(0.05) { // 50ms time step
    // Initialize matrices based on a constant velocity model (simplest)
    // State: [az, el, az_vel, el_vel]
    x_hat.resize(4);
    x_hat.setZero();

    P.resize(4, 4);
    P.setIdentity();
    P *= 1000; // Large initial uncertainty

    F.resize(4, 4);
    F << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;

    H.resize(2, 4); // We measure az and el
    H << 1, 0, 0, 0,
         0, 1, 0, 0;
8
    Q.resize(4, 4); // Process noise (how much drone's velocity/acceleration changes)
    Q << 0.1, 0, 0, 0,
         0, 0.1, 0, 0,
         0, 0, 0.1, 0,
         0, 0, 0, 0.1;

    R.resize(2, 2); // Measurement noise (Lidar accuracy)
    R << 1.0, 0,
         0, 1.0;

    I.resize(4, 4);
    I.setIdentity();
}

void KalmanFilter::initializeState(double initial_az, double initial_el, double initial_az_vel, double initial_el_vel) {
    x_hat << initial_az, initial_el, initial_az_vel, initial_el_vel;
    P = I * 10; // Smaller initial uncertainty after first detection
}

void KalmanFilter::predict() {
    x_hat = F * x_hat;
    P = F * P * F.transpose() + Q;
}

void KalmanFilter::update(const std::vector<double>& measurement) {
    Eigen::VectorXd z(2);
    z << measurement[0], measurement[1]; // Convert std::vector to Eigen::Vector

    Eigen::VectorXd y = z - H * x_hat; // Innovation
    Eigen::MatrixXd S = H * P * H.transpose() + R; // Innovation covariance
    Eigen::MatrixXd K = P * H.transpose() * S.inverse(); // Kalman gain

    x_hat = x_hat + K * y;
    P = (I - K * H) * P;
}

std::vector<double> KalmanFilter::getState() const {
    return {x_hat(0), x_hat(1), x_hat(2), x_hat(3)};
}
