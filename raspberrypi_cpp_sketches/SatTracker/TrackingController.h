#pragma once
#include "SensorManager.h"
#include "MotorController.h"
#include "KalmanFilter.h"
#include <memory>

struct TargetState {
    double x, y, z;           // Position in meters
    double vx, vy, vz;        // Velocity in m/s
    double ax, ay, az;        // Acceleration in m/s²
    double confidence;        // Track confidence [0-1]
    bool valid;              // Track validity
    double last_update;      // Timestamp of last update
};

struct TrackingParameters {
    double acquisition_range;     // Maximum acquisition range (m)
    double min_target_velocity;   // Minimum target velocity (m/s)
    double max_acceleration;      // Maximum expected acceleration (m/s²)
    double noise_threshold;       // Sensor noise threshold
    double prediction_horizon;    // Prediction time horizon (s)
};

class TrackingController {
private:
    SensorManager* sensor_manager;
    MotorController* motor_controller;
    std::unique_ptr<KalmanFilter> kalman_filter;

    TargetState current_target;
    TrackingParameters params;

    enum class TrackingState {
        SEARCHING,
        ACQUIRING,
        TRACKING,
        LOST
    } current_state;

    // Tracking algorithms
    bool detectTarget(const LidarReading& reading);
    void initializeTrack(const LidarReading& reading);
    void updateTrack(const LidarReading& reading, double dt);
    void predictTargetPosition(double dt);
    std::pair<double, double> calculatePointingAngles(const TargetState& target);

    // Control algorithms
    void executePointingCommand(double pan_angle, double tilt_angle);
    bool validateTrack(const TargetState& target);
    void handleTrackLoss();

public:
    TrackingController(SensorManager* sensor_mgr, MotorController* motor_ctrl);
    ~TrackingController();

    void update(double delta_time);
    void setTrackingParameters(const TrackingParameters& params);
    TargetState getCurrentTarget() const;
    TrackingState getCurrentState() const;
    void resetTracking();
};
