# Drone Tracking System

A simplified Raspberry Pi-based drone tracking system using LiDAR, stepper motor, and servo for azimuth and elevation control.

## Hardware Components

- **Raspberry Pi**: Main control unit
- **LiDAR Sensor**: TFmini-S for distance measurements
- **Stepper Motor**: Controls azimuth (horizontal rotation) with DRV8825 driver
- **Servo Motor**: Controls elevation (vertical tilt)
- **GPIO Connections**: Direct control of motor drivers

## System Overview

The system implements a 4-step drone tracking process:

### 1. Environmental Calibration
- Scans the environment systematically using stepper (360° azimuth) and servo (0-70° elevation)
- Collects baseline distance measurements at 2° azimuth and 5° elevation increments
- Saves calibration data to CSV file for reference

### 2. Search Scanning
- Performs systematic search scans across the environment
- Uses forward and reverse sweeps to detect anomalies
- Compares current LiDAR readings against calibration baseline
- Detects potential drones when distance is significantly shorter than expected

### 3. Tracking Mode
- When a target is detected, switches to tracking mode
- Scans around the last detected point in a pattern (center, left/right, up/down, diagonals)
- Updates tracking position when new detections are found
- Continues tracking the moving target

### 4. Continuous Operation
- System runs indefinitely, switching between search and tracking modes
- Automatically returns to search mode if target is lost
- Maintains last known position for efficient re-acquisition

## File Structure

- **`main.py`**: Main program implementing the 4-step tracking process
- **`constants.py`**: System configuration and pin definitions
- **`stepper_setup.py`**: GPIO setup for stepper motor control
- **`move_motors.py`**: Motor control functions for positioning
- **`lidar_reader.py`**: LiDAR data acquisition in background thread
- **`calibration.py`**: Environmental scanning and calibration
- **`scanning.py`**: Search and tracking scanning algorithms
- **`anomaly_detection.py`**: Simple anomaly detection logic

## Key Features

- **Real-time Processing**: LiDAR data processed in background thread
- **Efficient Scanning**: Optimized motor movements with shortest path calculation
- **Robust Detection**: Simple but effective anomaly detection based on distance thresholds
- **Position Tracking**: Maintains accurate motor position tracking
- **Error Handling**: Graceful handling of motor limits and sensor errors

## Configuration

Key parameters in `constants.py`:
- Motor pin assignments (BCM numbering)
- LiDAR serial port and baud rate
- Stepper motor steps per revolution
- Servo angle limits (0-70°)
- Anomaly detection threshold factor (0.6)

## Usage

1. Ensure pigpio daemon is running: `sudo pigpiod`
2. Connect hardware according to pin definitions
3. Run the program: `python3 main.py`
4. System will automatically start calibration, then begin search and tracking

## Dependencies

- RPi.GPIO
- pigpio
- pyserial
- Standard Python libraries (time, threading, queue, csv, datetime)

## Notes

- System designed for simplicity and reliability over complex algorithms
- Removed Kalman filtering and advanced tracking algorithms for easier maintenance
- Focuses on basic but effective drone detection and tracking
- Calibration data saved with timestamps for analysis
