# Drone Tracking System

A real-time drone tracking and trajectory prediction system built with React, Three.js, and Kalman filtering. This system can receive GPS coordinates from an API (or simulate them) and convert them to Cartesian coordinates for visualization and prediction.

## Features

### 🚁 **Real-time Drone Tracking**
- GPS coordinate reception and conversion to Cartesian coordinates
- Real-time position updates with 3D visualization
- Support for both manual input and API data streams

### 🧮 **Kalman Filter Implementation**
- 3D position and velocity tracking
- Trajectory prediction with configurable time steps
- Noise reduction and smoothing of GPS measurements
- Future position estimation

### 🎮 **Simulation & Control**
- Built-in drone simulator with realistic movement patterns
- Flight pattern generation (Circle, Figure-8, Square, Random)
- Manual control via throttle, pitch, roll, and yaw inputs
- Environmental factors simulation (wind, turbulence)

### 🌍 **3D Visualization**
- Interactive 3D scene with Earth representation
- Drone position markers and trajectory paths
- Ground station positioning
- Measured vs. predicted trajectory comparison
- Orbit controls for scene navigation

### 🔧 **Coordinate System Support**
- GPS (Latitude, Longitude, Altitude) to ECEF conversion
- ECEF to ENU coordinate transformations
- WGS84 ellipsoid parameters
- Scalable visualization coordinates

## Getting Started

### Prerequisites
- Node.js (v14 or higher)
- npm or yarn

### Installation
```bash
# Clone the repository
git clone <repository-url>
cd sat-track

# Install dependencies
npm install

# Start the development server
npm run dev
```

### Usage

#### 1. **Manual GPS Input**
- Enter latitude, longitude, and altitude in the input fields
- Click "Add Measurement" to add a new GPS point
- The system will convert coordinates and update the visualization

#### 2. **Simulation Mode**
- Click "Start Simulation" to begin automatic drone movement
- Choose from predefined flight patterns:
  - **Circle**: Circular flight path
  - **Figure-8**: Figure-eight pattern
  - **Square**: Square flight path
  - **Random**: Random exploration pattern

#### 3. **Manual Control**
- Use the sliders to control drone movement:
  - **Throttle**: Vertical movement (up/down)
  - **Pitch**: Forward/backward tilt
  - **Roll**: Left/right tilt
  - **Yaw**: Rotation around vertical axis
- Click "Apply Control" to execute the commands

#### 4. **Ground Station**
- Set the ground station position using the input coordinates
- The ground station will appear as a cylinder in the 3D scene

#### 5. **System Control**
- **Reset System**: Clear all data and reset the Kalman filter
- **Stop Simulation**: Halt automatic drone movement
- Monitor system status and measurement count

## Technical Details

### Architecture
- **Frontend**: React with React Three Fiber for 3D graphics
- **3D Engine**: Three.js for graphics rendering
- **Filtering**: Custom 3D Kalman filter implementation
- **Coordinates**: GPS ↔ ECEF ↔ ENU conversion utilities

### Kalman Filter
The system implements a 6-state Kalman filter tracking:
- Position: x, y, z (3D coordinates)
- Velocity: vx, vy, vz (3D velocity)

### Coordinate Conversion
- **GPS to ECEF**: Converts geographic coordinates to Earth-Centered, Earth-Fixed
- **ECEF to ENU**: Converts to East-North-Up local coordinate system
- **Scaling**: Automatic scaling for visualization purposes

### API Integration
The system is designed to integrate with external drone APIs:
- Real-time GPS coordinate reception
- Telemetry data processing
- Command transmission capabilities

## File Structure

```
src/
├── components/
│   ├── DroneTracker.jsx      # Main tracking component
│   └── DroneTracker.css      # Component styling
├── utils/
│   ├── KalmanFilter.js       # 3D Kalman filter implementation
│   ├── CoordinateConverter.js # Coordinate system utilities
│   └── DroneSimulator.js     # Drone simulation and API mock
├── App.jsx                   # Main application component
└── main.jsx                  # Application entry point
```

## Configuration

### Kalman Filter Parameters
- **Process Noise**: Configurable noise levels for position and velocity
- **Measurement Noise**: GPS measurement uncertainty settings
- **Prediction Steps**: Number of future trajectory points (default: 20)
- **Time Step**: Prediction interval in seconds (default: 0.5s)

### Visualization Settings
- **Scale Factor**: Coordinate scaling for 3D display (default: 0.001)
- **Max Trajectory Points**: Maximum displayed trajectory length (default: 100)
- **Camera Position**: Initial 3D scene viewpoint

## Future Enhancements

- [ ] Real-time API integration for live drone data
- [ ] Multiple drone tracking support
- [ ] Advanced flight path planning algorithms
- [ ] Weather condition integration
- [ ] Export trajectory data to various formats
- [ ] Mobile-responsive design improvements
- [ ] Performance optimization for large datasets

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- React Three Fiber community for 3D graphics support
- Three.js team for the 3D graphics library
- Kalman filter research and implementation references
