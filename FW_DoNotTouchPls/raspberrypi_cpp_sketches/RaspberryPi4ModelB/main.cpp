#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include "LidarDriver.h"
#include "SerialCommunicator.h"
#include "Tracker.h"
#include "KalmanFilter.h" // Assuming you implement this

// Define your serial port for Arduino
const std::string ARDUINO_PORT = "/dev/ttyACM0"; // Check your Pi for the correct port
const int BAUD_RATE = 115200; // Match Arduino baud rate

int main() {
    std::cout << "Satellite Tracker Simulation - Raspberry Pi Main Program" << std::endl;

    // 1. Initialize Serial Communication with Arduino
    SerialCommunicator arduinoSerial(ARDUINO_PORT, BAUD_RATE);
    if (!arduinoSerial.openPort()) {
        std::cerr << "Failed to open serial port to Arduino!" << std::endl;
        return 1;
    }
    std::cout << "Serial connection to Arduino established." << std::endl;

    // 2. Initialize LiDAR Driver (assuming serial for TFmini-S)
    LidarDriver lidar("TFMINI-S", "/dev/ttyAMA0", BAUD_RATE); // Check your Pi for the correct UART port
    if (!lidar.init()) {
        std::cerr << "Failed to initialize LiDAR sensor!" << std::endl;
        arduinoSerial.closePort();
        return 1;
    }
    std::cout << "LiDAR sensor initialized." << std::endl;

    // 3. Initialize Tracker with Kalman Filter
    Tracker droneTracker;
    KalmanFilter kf; // Initialize with appropriate state and measurement models for angles/velocities

    // Initial "Home" or "Search" position (angles in degrees)
    double currentAzimuth = 0.0;
    double currentElevation = 0.0;
    droneTracker.sendCommandToArduino(arduinoSerial, currentAzimuth, currentElevation);
    std::cout << "Moving to home position (0,0)." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Give motors time to move

    bool trackingActive = false;
    int searchStep = 0;
    int searchDirection = 1; // 1 for right, -1 for left for azimuth sweep

    while (true) {
        // Read LiDAR distance
        int distance = lidar.getDistance(); // Distance in cm
        if (distance > 0 && distance < 10000) { // Valid distance, adjust max range as per TFmini-S
            std::cout << "LiDAR Distance: " << distance << " cm" << std::endl;

            // Simple detection: if something is within 5 meters (500cm)
            if (distance < 500 && !trackingActive) {
                std::cout << "Drone detected! Entering tracking mode." << std::endl;
                trackingActive = true;
                // Initialize Kalman filter state with current estimated position
                // kf.initializeState(currentAzimuth, currentElevation, 0, 0); // Az, El, Az_vel, El_vel
            }
        } else {
            std::cout << "LiDAR Out of Range / Error." << std::endl;
            if (trackingActive) {
                // Potential loss of target, try to re-acquire
                std::cout << "Target lost! Reverting to search or re-acquisition pattern." << std::endl;
                trackingActive = false;
                // Implement re-acquisition logic (e.g., small spiral search around last known point)
            }
        }

        if (trackingActive) {
            // Tracking Logic
            // Simulate current measured angles based on LiDAR distance and assumed geometry
            // In a real setup, you'd need calibration to map distance and pan/tilt positions
            // to actual drone coordinates relative to the sensor.
            // For a simple demo, we can assume the LiDAR is centered and just move towards closer objects.
            // This is a simplification; ideally, you'd calculate angular error from LiDAR data.

            // Dummy angular error calculation for demonstration
            double azError = 0; // Degrees
            double elError = 0; // Degrees
            if (distance > 0 && distance < 500) {
                 // Example: if distance is less, perhaps the drone is moving away or towards the edge
                 // This requires a more sophisticated mapping of LiDAR data to angular displacement
                 // For now, let's assume we want to center the drone, so error should tend to zero.
                 // This is where real angular position of lidar relative to desired target would come in.
                 // A simple approach for the demo: if distance is low, the drone is "in front"
                 // If you had multiple lidar readings or knew the FOV of the lidar + the drone's size, you could infer.

                 // For actual angular error, you need a way to determine *where* in the FOV the drone is.
                 // The TFmini-S is a single-point lidar, so it only gives distance.
                 // To get angular error, you'd typically need:
                 // 1. A wider FOV sensor (e.g., camera with object detection)
                 // 2. Or, a scanning lidar (like a 2D lidar)
                 // 3. Or, you assume the current motor position *is* the target, and if distance is stable, you're good.
                 // To simulate angular error for the demo: assume if distance decreases, the drone is moving,
                 // and we apply a simple proportional control to recenter. This is a simplification.

                 // Let's assume you've calculated a desired target_azimuth and target_elevation
                 // based on some external knowledge or a more advanced sensor.
                 // For this demo, let's just make it "seek" a minimum distance.
                 // This is *not* true angular tracking, but a demonstration of control.
                 // A better simulation would involve knowing the drone's "true" position and calculating error.

                // Instead of angular error from LiDAR alone, you'd need a reference.
                // For a *true* tracking demo, the LiDAR is only distance. You need a camera for angular position
                // or multiple LiDARs/scanning LiDAR.
                // Let's pivot slightly for the "sell": The LiDAR is for range, and the servo/stepper are
                // commanded to sweep based on predicted trajectory or external commands.
                // For the "drone tracking" aspect, we'll assume the drone is mostly stationary within range,
                // and the LiDAR provides the "detection".

                // For a true drone tracker, you'd use a camera (Raspberry Pi Camera Module) with OpenCV
                // for object detection (e.g., color-based, or simple YOLO if Pi 4 can handle it)
                // to get the pixel coordinates of the drone. Then map pixel error to angular error.
                // The LiDAR would confirm range.

                // *Simplified Tracking Logic for Lidar Only (Focus on PID for movement)*
                // This is where the Kalman Filter for *prediction* comes in.
                // Imagine we have a *target_azimuth* and *target_elevation* from an external source or
                // a more complex detection system (e.g. a simulated drone flight path).
                // Or, if we assume the LiDAR always points at the center of the drone, and we are just
                // trying to minimize distance by moving towards it. This is tricky with single-point LiDAR.

                // Let's simulate a target moving and our system trying to follow based on some angular estimates.
                // This will be a *concept* of tracking, rather than direct angular tracking from single-point LiDAR.

                // If you want actual angular tracking with TFmini-S, you would need to:
                // 1. Sweep the LiDAR to create a "point cloud" (or a line scan).
                // 2. Identify the closest point within that sweep as your target.
                // 3. Calculate the angle to that closest point. This is much more complex.

                // *Revised Simple Tracking Logic (LiDAR for detection, PID for smooth movement based on a simple "error"):*
                // Let's assume our current aim is `currentAzimuth`, `currentElevation`.
                // If we get a valid LiDAR distance, we can infer that the drone is "in front".
                // We'll then use the Kalman filter to smooth our *own* estimated position, and our PID
                // will try to keep the LiDAR pointing at *some* point that minimizes range.

                // The "error" for PID will be based on how much our estimated target position differs from our current pointing direction.
                // Here, the Kalman filter would ideally estimate the drone's *true* azimuth and elevation.
                // Let's assume `kf.predict()` gives us a predicted `predicted_az` and `predicted_el`.
                // And `kf.update(current_measured_az, current_measured_el)` updates the state.
                // With a single-point LiDAR, `current_measured_az` and `current_measured_el` would be derived from
                // the *current pan-tilt angles* if a detection is made *at that angle*.

                // For a robust demo, you'd need to simulate the drone's path or use another sensor (camera)
                // to get angular data. Given your current sensors, a camera is missing for true angular error.
                // Let's make the "tracking" more about maintaining the *closest known point* and moving smoothly.

                // **Correct Approach for Tracking with Single-Point LiDAR:**
                // The single-point LiDAR (TFmini-S) provides *distance* at the point it's looking.
                // To track, you need to find *where* the object is angularly.
                // You can achieve this by:
                // a) **Scanning and finding the minimum distance:** Systematically sweep the pan-tilt, record distances, find the angle with the minimum distance (closest point). This becomes your target.
                // b) **Using a camera:** Raspberry Pi can easily integrate with a camera module. This is by far the *most effective and quickest* way to find and track visually, using OpenCV for object detection (e.g., color, bounding box, simple AI model for "drone"). The LiDAR would then be used for range verification.

                **Given the constraint of TFmini-S being your *only* sensor for drone "detection" beyond initial setup, option (a) is the only way for the LiDAR to provide angular data. This means your tracking will always involve small, rapid scans.**

                // Let's assume a simplified scanning approach for tracking:
                // Perform a small, quick scan around the current estimated position to find the minimum distance.
                // This minimal distance corresponds to the center of the drone.
                // Let's define `scan_range = 2.0` degrees (e.g., -1 to +1 degree relative to current)
                // And `scan_steps = 5` steps.

                double best_az = currentAzimuth;
                double best_el = currentElevation;
                int min_dist = 99999; // A very large initial distance

                // Small scan around current estimated position
                for (double d_az = -scan_range; d_az <= scan_range; d_az += scan_range / scan_steps) {
                    for (double d_el = -scan_range; d_el <= scan_range; d_el += scan_range / scan_steps) {
                        double scan_az = currentAzimuth + d_az;
                        double scan_el = currentElevation + d_el;

                        droneTracker.sendCommandToArduino(arduinoSerial, scan_az, scan_el);
                        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Allow motors to move
                        int scan_dist = lidar.getDistance();

                        if (scan_dist > 0 && scan_dist < min_dist) {
                            min_dist = scan_dist;
                            best_az = scan_az;
                            best_el = scan_el;
                        }
                    }
                }

                if (min_dist < 500) { // Still detecting the drone
                    // Use Kalman filter to smooth the `best_az` and `best_el` measurements
                    // You'd feed (best_az, best_el) as measurements to the Kalman filter
                    kf.predict(); // Predict next state
                    kf.update({best_az, best_el}); // Update with measurement (assuming simple 2D state for angles)

                    // Get the estimated (smoothed) position from Kalman Filter
                    double estimated_az = kf.getState()[0]; // Azimuth from KF state
                    double estimated_el = kf.getState()[1]; // Elevation from KF state

                    // Now calculate PID control based on deviation from estimated position
                    // The PID error is `(estimated_az - currentAzimuth)` and `(estimated_el - currentElevation)`
                    // Or, more accurately, `(target_azimuth - currentAzimuth)` where target_azimuth comes from KF prediction
                    azError = estimated_az - currentAzimuth;
                    elError = estimated_el - currentElevation;

                    // PID calculation (simplified, would be in Tracker.cpp)
                    double pidAzAdjust = droneTracker.calculatePID(azError, "azimuth");
                    double pidElAdjust = droneTracker.calculatePID(elError, "elevation");

                    // Update current angles
                    currentAzimuth += pidAzAdjust;
                    currentElevation += pidElAdjust;

                    // Clamp angles to valid ranges (e.g., azimuth 0-360, elevation -30 to 90)
                    currentAzimuth = std::fmod(currentAzimuth, 360.0);
                    if (currentAzimuth < 0) currentAzimuth += 360.0;
                    currentElevation = std::max(-30.0, std::min(90.0, currentElevation)); // Example range

                    // Send new angles to Arduino
                    droneTracker.sendCommandToArduino(arduinoSerial, currentAzimuth, currentElevation);
                    std::cout << "Tracking: Moving to Az: " << currentAzimuth << ", El: " << currentElevation << std::endl;

                    // Log data (for presentation)
                    // droneTracker.logData(currentAzimuth, currentElevation, distance, estimated_az, estimated_el);

                } else {
                    std::cout << "Drone lost during mini-scan. Reverting to search." << std::endl;
                    trackingActive = false;
                }

            }
        } else {
            // Search Logic (Slow Sweep)
            // Example: Simple horizontal sweep for search
            currentAzimuth += searchDirection * 0.5; // Sweep 0.5 degrees at a time
            if (currentAzimuth > 45.0 || currentAzimuth < -45.0) { // Sweep range -45 to +45
                searchDirection *= -1; // Reverse direction
                currentAzimuth += searchDirection * 0.5; // Adjust for immediate turn
                currentElevation += 1.0; // Increment elevation slightly after each sweep
                if (currentElevation > 30.0) currentElevation = 0.0; // Reset elevation if max reached
            }
            droneTracker.sendCommandToArduino(arduinoSerial, currentAzimuth, currentElevation);
            std::cout << "Searching: Moving to Az: " << currentAzimuth << ", El: " << currentElevation << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Slower search
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Main loop delay
    }

    arduinoSerial.closePort();
    return 0;
}
