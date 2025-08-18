# Drone Detection and Tracking System

A comprehensive Raspberry Pi-based system for detecting and tracking drones using LiDAR sensors, stepper motors, and servo motors.

## 🎯 System Overview

This system provides a complete solution for drone detection and tracking with the following capabilities:

- **Hardware Initialization**: GPIO setup, stepper motor control, servo motor control, and LiDAR sensor integration
- **Environment Calibration**: Systematic calibration of the scanning environment
- **Drone Search**: Multiple search patterns (spiral, grid, concentric, zigzag) to find drones
- **Point-by-Point Tracking**: Predictive tracking with velocity estimation and movement prediction
- **Data Processing**: TLE coordinate data, position tracking, velocity calculation, and movement prediction
- **Comprehensive Output**: JSON, CSV, and summary data formats

## 🏗️ System Architecture

### Core Components

1. **`firmware_StpInz.py`** - Hardware initialization and control
   - GPIO setup for stepper motor driver (DRV8825)
   - Servo motor control via pigpio
   - LiDAR sensor setup and data collection
   - Motor movement functions

2. **`background_scan.py`** - Environment scanning and calibration
   - Systematic environment calibration
   - Anomaly detection algorithms
   - Scanning sequences and data collection

3. **`find_first_position.py`** - Drone search algorithms
   - Multiple search patterns (spiral, grid, concentric, zigzag)
   - Systematic area coverage
   - Position detection and validation

4. **`track_point_by_point.py`** - Predictive tracking system
   - Velocity calculation and smoothing
   - Movement prediction algorithms
   - Point-by-point tracking with search recovery

5. **`data_processing.py`** - Data processing and output
   - TLE coordinate processing
   - Position and velocity analysis
   - Movement prediction
   - Multiple output formats (JSON, CSV, TLE)

6. **`main.py`** - System orchestration
   - Complete system workflow
   - State management
   - Error handling and recovery
   - Data logging and session management

## 🛠️ Hardware Requirements

### Required Components
- **Raspberry Pi** (3B+ or 4B recommended)
- **TFmini-S LiDAR Sensor** (or compatible)
- **Stepper Motor** with DRV8825 driver board
- **Servo Motor** (for elevation control)
- **Power Supply** (adequate for all components)
- **Mounting Hardware** (for sensor and motor assembly)

### GPIO Pin Configuration
```
Stepper Motor (DRV8825):
- DIR_PIN: GPIO 26
- STEP_PIN: GPIO 13
- ENABLE_PIN: GPIO 23
- RESET_PIN: GPIO 19
- M0_PIN: GPIO 0
- M1_PIN: GPIO 5
- M2_PIN: GPIO 6

Servo Motor:
- SERVO_PIN: GPIO 2

LiDAR Sensor:
- Serial Port: /dev/ttyS0
- Baud Rate: 115200
```

## 📦 Installation

### 1. System Dependencies
```bash
# Update system
sudo apt-get update
sudo apt-get upgrade

# Install required packages
sudo apt-get install python3-pip python3-dev
sudo apt-get install pigpio python3-pigpio
sudo apt-get install python3-rpi.gpio
```

### 2. Python Dependencies
```bash
# Install Python packages
pip3 install -r requirements.txt

# Or install individually
pip3 install RPi.GPIO pigpio pyserial numpy scipy ephem pandas
```

### 3. Hardware Setup
1. Connect stepper motor to DRV8825 driver board
2. Connect servo motor to GPIO 2
3. Connect LiDAR sensor to serial port
4. Ensure proper power supply for all components
5. Mount sensors and motors securely

### 4. System Configuration
1. Verify GPIO pin assignments in each module
2. Adjust LiDAR serial port if needed
3. Configure motor parameters (steps per revolution, etc.)
4. Set appropriate search and tracking parameters

## 🚀 Usage

### Quick Start
```bash
# Run the complete system
python3 main.py
```

### Individual Module Testing
```bash
# Test firmware initialization
python3 firmware_StpInz.py

# Test background scanning
python3 background_scan.py

# Test drone search
python3 find_first_position.py

# Test tracking
python3 track_point_by_point.py

# Test data processing
python3 data_processing.py
```

### Configuration Options

#### Search Patterns
- `spiral`: Spiral pattern from center outward
- `grid`: Grid pattern covering entire area
- `concentric`: Concentric circles from center
- `zigzag`: Zigzag pattern covering area

#### Tracking Parameters
- `MAX_TRACKING_POINTS`: Maximum positions to track (default: 100)
- `TRACKING_INTERVAL`: Time between tracking attempts (default: 1.0s)
- `MAX_SEARCH_TIME`: Maximum search time (default: 300s)

#### Output Formats
- `json`: JSON data files
- `csv`: CSV data files
- `tle`: TLE format files
- `summary`: Console summary output

## 📊 System Workflow

### 1. Initialization Phase
- GPIO pin setup
- Motor driver configuration
- LiDAR sensor initialization
- System self-test

### 2. Calibration Phase
- Environment mapping
- Reference distance collection
- Anomaly detection baseline
- Data validation

### 3. Search Phase
- Systematic area coverage
- Multiple search patterns
- Anomaly detection
- Position validation

### 4. Tracking Phase
- Velocity calculation
- Movement prediction
- Point-by-point tracking
- Search recovery if target lost

### 5. Processing Phase
- Data analysis
- TLE coordinate processing
- Velocity and trajectory analysis
- Multiple format output

## 📈 Data Output

### File Formats
- **JSON**: Complete structured data
- **CSV**: Tabular data for analysis
- **TLE**: Satellite/drone orbital elements
- **Session**: Complete session summary

### Data Fields
- **Position**: Azimuth, elevation, distance
- **Velocity**: Azimuth, elevation, distance velocities
- **Timestamps**: Detection and processing times
- **Confidence**: Position confidence scores
- **Status**: Tracking status (tracking/searching/lost)

## 🔧 Troubleshooting

### Common Issues

#### Hardware Issues
- **GPIO errors**: Check pin connections and permissions
- **Motor not moving**: Verify power supply and driver connections
- **LiDAR not responding**: Check serial port and baud rate

#### Software Issues
- **Import errors**: Install missing dependencies
- **Permission errors**: Run with appropriate permissions
- **Memory issues**: Optimize tracking parameters

### Debug Mode
```bash
# Enable debug output
export DEBUG=1
python3 main.py
```

### Log Files
- System logs are saved to timestamped files
- Error logs include detailed error information
- Session data includes complete execution history

## 🔒 Safety Considerations

### Hardware Safety
- Ensure proper power supply ratings
- Use appropriate mounting hardware
- Protect against environmental factors
- Regular maintenance and inspection

### Software Safety
- Implement emergency stop functionality
- Monitor system resources
- Implement error recovery mechanisms
- Regular data backup

## 📝 Development

### Code Structure
```
├── main.py                 # Main system orchestration
├── firmware_StpInz.py      # Hardware initialization
├── background_scan.py      # Environment scanning
├── find_first_position.py  # Drone search algorithms
├── track_point_by_point.py # Predictive tracking
├── data_processing.py      # Data processing and output
├── requirements.txt        # Python dependencies
├── README.md              # This file
└── Firmware/              # Original firmware files
```

### Contributing
1. Follow existing code style
2. Add comprehensive documentation
3. Include error handling
4. Test thoroughly before submission

### Testing
```bash
# Run tests
python3 -m pytest tests/

# Run specific module tests
python3 main_test.py
```

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🤝 Support

For support and questions:
- Check the troubleshooting section
- Review the documentation
- Open an issue on the project repository

## 🔄 Version History

- **v1.0.0**: Initial release with complete system
- **v1.1.0**: Added multiple search patterns
- **v1.2.0**: Enhanced tracking algorithms
- **v1.3.0**: Improved data processing and output

---

**Note**: This system is designed for educational and research purposes. Ensure compliance with local regulations regarding drone detection and tracking.