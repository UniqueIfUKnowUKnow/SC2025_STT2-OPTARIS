#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include "SensorManager.h"
#include "TrackingController.h"
#include "MotorController.h"

// Global system state
volatile bool system_running = true;
std::unique_ptr<SensorManager> sensor_mgr;
std::unique_ptr<TrackingController> tracker;
std::unique_ptr<MotorController> motor_ctrl;

void signal_handler(int signum) {
    std::cout << "Shutting down satellite tracker..." << std::endl;
    system_running = false;
}

int main() {
    // Initialize signal handling
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize WiringPi library
    if (wiringPiSetup() < 0) {
        std::cerr << "Failed to initialize WiringPi" << std::endl;
        return -1;
    }

    std::cout << "Professional Satellite Tracking System v1.0" << std::endl;
    std::cout << "Initializing subsystems..." << std::endl;

    try {
        // Initialize system components
        sensor_mgr = std::make_unique<SensorManager>("/dev/serial0", 115200);
        motor_ctrl = std::make_unique<MotorController>(0x08); // I2C address
        tracker = std::make_unique<TrackingController>(sensor_mgr.get(), motor_ctrl.get());

        // System calibration sequence
        std::cout << "Performing system calibration..." << std::endl;
        motor_ctrl->calibrate();
        sensor_mgr->calibrate();

        // Main control loop
        std::cout << "Entering tracking mode..." << std::endl;
        auto last_update = std::chrono::high_resolution_clock::now();

        while (system_running) {
            auto current_time = std::chrono::high_resolution_clock::now();
            auto delta_time = std::chrono::duration<double>(current_time - last_update).count();

            // Update sensor readings
            sensor_mgr->update();

            // Execute tracking algorithms
            tracker->update(delta_time);

            // Control loop timing (100Hz update rate)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            last_update = current_time;
        }

    } catch (const std::exception& e) {
        std::cerr << "System error: " << e.what() << std::endl;
        return -1;
    }

    // Graceful shutdown
    std::cout << "System shutdown complete." << std::endl;
    return 0;
}
