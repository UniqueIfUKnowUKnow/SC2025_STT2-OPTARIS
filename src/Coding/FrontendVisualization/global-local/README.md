# Drone Tracker Digital Twin - Enhanced Version 2.0.0

## Overview

The Drone Tracker Digital Twin is a comprehensive satellite tracking and LiDAR monitoring system that provides real-time 3D visualization of Earth, satellites, and local LiDAR tracking operations. This enhanced version includes a professional start screen, stationary UI panels, improved 3D rendering, and comprehensive system state management.

## Key Features

### 🚀 **New Start Screen**
- Professional application entry point with animated background
- Real-time date and time display
- System status indicators
- Clean, modern interface before launching the main application

### 🌍 **Global View (Earth Orbit)**
- **Fixed Ground Station Position**: Sofia Tech Park marker now correctly "pinned" to Earth's surface
- **Visible Optaris Orbit**: Full elliptical orbit path rendered around the globe
- **Satellite Tracking**: Real-time tracking of Optaris and ISS satellites
- **Removed Confusing Elements**: Measured position marker no longer appears in global view

### 🔍 **Local View (LiDAR Tracker)**
- **Connected Scanner Beam**: Laser beam now properly connects to target during tracking
- **Predicted Orbit Visualization**: Shows calculated orbital path based on LiDAR measurements
- **Enhanced History Display**: Last 5 measured positions with smooth fade-out animation
- **Live Telemetry**: Real-time display of pan/tilt angles, signal strength, and distance

### 🎛️ **Stationary UI Layout**
- **Left Side Panels**: System Control, System Status, and Target Data
- **Right Side Panels**: Orbital Parameters and Visual Legend
- **Fixed Positioning**: UI panels remain stationary during 3D scene interaction
- **Automatic Legend Switching**: Legend content changes based on current view

### ⏰ **Date & Time Display**
- **Fixed Bottom Display**: Current date and time always visible
- **Real-time Updates**: Updates every second automatically
- **Professional Styling**: Integrated with the overall design theme

## System Architecture

### Frontend (React Application)
```
unified-tracker/
├── src/
│   ├── components/
│   │   ├── StartScreen.jsx          # New entry screen
│   │   ├── GlobalView.jsx           # Enhanced Earth view
│   │   ├── LocalView.jsx            # Enhanced LiDAR view
│   │   ├── SystemHUD.jsx            # Stationary UI panels
│   │   ├── RealisticEarth.jsx       # 3D Earth model
│   │   └── Starfield.jsx            # Background stars
│   ├── store/
│   │   └── useUnifiedStore.js       # Central data management
│   ├── services/
│   │   └── SensorWebSocket.js       # WebSocket communication
│   ├── utils/
│   │   └── geo.js                   # Geospatial calculations
│   ├── App.jsx                      # Main application logic
│   └── main.jsx                     # Application entry point
├── public/
│   └── textures/
│       └── earth_daymap.jpg         # Earth texture
└── package.json                     # Dependencies and scripts
```

### Backend (Python WebSocket Server)
```
Raspberry Pi/
└── main_unified.py                  # Enhanced backend with state machine
```

## Installation & Setup

### Prerequisites
- Node.js 16+ and npm
- Python 3.8+ with websockets library
- Modern web browser (Chrome, Firefox, Safari, Edge)

### Frontend Setup
1. **Navigate to the frontend directory:**
   ```bash
   cd unified-tracker
   ```

2. **Install dependencies:**
   ```bash
   npm install
   ```

3. **Start the development server:**
   ```bash
   npm run dev
   ```

4. **Open your browser** and navigate to `http://localhost:5173`

### Backend Setup
1. **Navigate to the backend directory:**
   ```bash
   cd "Raspberry Pi"
   ```

2. **Install Python dependencies:**
   ```bash
   pip install websockets
   ```

3. **Run the backend server:**
   ```bash
   python main_unified.py --simulate
   ```

## Usage Instructions

### Starting the Application
1. **Launch the frontend** - You'll see the professional start screen
2. **Click "Start Application"** - This launches the main tracking interface
3. **Connect to backend** - Enter your Raspberry Pi's IP address in the WebSocket URL field
4. **Click "Connect"** - Establish connection to the backend

### Controlling the System
- **Start Button**: Begins the tracking simulation
- **Stop Button**: Pauses the current operation
- **Reset Button**: Clears all data and returns to standby
- **Switch View Button**: Toggle between Global (Earth) and Local (LiDAR) views

### View Navigation
- **Global View**: 
  - Click and drag to rotate Earth
  - Scroll to zoom in/out
  - Click ground station marker to switch to Local View
- **Local View**:
  - Navigate around the LiDAR sensor
  - Observe scanner beam movement
  - View target tracking in real-time

## System States & Operations

### 1. **STANDBY**
- System is ready but inactive
- All sensors in idle mode
- Waiting for start command

### 2. **INITIALIZING**
- System startup and calibration
- Progress bar shows initialization status
- Automatic transition to background scan

### 3. **BACKGROUND_SCAN_IN_PROGRESS**
- LiDAR performs 360-degree environment scan
- Scanner beam follows circular pattern
- Progress bar shows scan completion percentage

### 4. **SEARCHING_FOR_TARGET**
- Systematic search pattern for specific targets
- Zig-zag scanning motion
- Random target detection simulation

### 5. **TRACKING_TARGET**
- Continuous target monitoring
- Real-time position updates
- Orbital parameter calculations
- Position history recording

## Data Flow

### Frontend → Backend
- **Start Command**: Initiates simulation
- **Stop Command**: Pauses current operation
- **Reset Command**: Clears all data

### Backend → Frontend
- **System Status**: Current operational state
- **Progress**: Operation completion percentage
- **Scanner Position**: Real-time beam direction
- **Live Telemetry**: Pan/tilt angles, signal strength, distance
- **Position History**: Target movement tracking
- **Orbital Parameters**: Calculated orbit characteristics

## Technical Specifications

### LiDAR Sensor
- **Range**: 0.1 to 12.0 meters
- **Angular Resolution**: 1.0 degree
- **Update Rate**: 20 Hz
- **Pan Range**: -180° to +180°
- **Tilt Range**: -90° to +90°

### Orbital Calculations
- **Minimum Data Points**: 10 positions required
- **Update Frequency**: Every 5 seconds
- **Parameters Calculated**:
  - Semi-major axis (orbit size)
  - Eccentricity (orbit shape)
  - Inclination (orbit tilt)
  - RAAN (ascending node)
  - Argument of periapsis
  - True anomaly

### WebSocket Communication
- **Protocol**: WebSocket over TCP
- **Port**: 8765 (configurable)
- **Data Format**: JSON
- **Update Rate**: 20 Hz

## Troubleshooting

### Common Issues

#### Frontend Won't Start
- **Check Node.js version**: Ensure you have Node.js 16+ installed
- **Clear npm cache**: Run `npm cache clean --force`
- **Delete node_modules**: Remove and reinstall with `npm install`

#### WebSocket Connection Fails
- **Verify IP Address**: Ensure the backend IP is correct
- **Check Firewall**: Port 8765 must be open
- **Backend Status**: Confirm the Python server is running

#### 3D Scene Not Rendering
- **Browser Compatibility**: Use Chrome, Firefox, or Edge
- **Graphics Drivers**: Update your graphics drivers
- **Hardware Acceleration**: Enable hardware acceleration in browser settings

#### Start Button Not Responding
- **Check Connection**: Ensure WebSocket status shows "connected"
- **Backend Logs**: Look for "Received start command" in backend console
- **Browser Console**: Check for JavaScript errors

### Debug Information

#### Frontend Debug
- Open browser Developer Tools (F12)
- Check Console tab for error messages
- Monitor Network tab for WebSocket activity

#### Backend Debug
- Watch terminal output for state changes
- Monitor WebSocket connection status
- Check for Python error messages

## Development & Customization

### Adding New Satellites
1. **Update TLE data** in `App.jsx`
2. **Add satellite colors** in the satellite initialization
3. **Modify orbit rendering** in `GlobalView.jsx`

### Customizing UI Panels
1. **Modify panel content** in `SystemHUD.jsx`
2. **Update styling** in `SystemHUD.css`
3. **Adjust positioning** in the CSS layout

### Extending Backend Features
1. **Add new states** to `SystemState` class
2. **Implement new simulation methods** in `DroneTrackerSystem`
3. **Extend data transmission** in `get_system_state()`

## Performance Considerations

### Frontend Optimization
- **3D Rendering**: Limit scene complexity for smooth performance
- **State Updates**: Minimize unnecessary re-renders
- **Memory Management**: Clean up 3D objects when switching views

### Backend Optimization
- **Update Frequency**: Balance between responsiveness and resource usage
- **Data Storage**: Limit position history size
- **WebSocket Management**: Handle multiple client connections efficiently

## Security Notes

### Network Security
- **Firewall Configuration**: Restrict access to WebSocket port
- **Network Isolation**: Run on isolated network when possible
- **Access Control**: Implement authentication if needed

### Data Privacy
- **Local Processing**: All calculations performed locally
- **No External APIs**: System operates independently
- **Configurable Logging**: Adjust logging verbosity as needed

## Support & Maintenance

### Regular Maintenance
- **Update Dependencies**: Keep npm packages and Python libraries current
- **Monitor Logs**: Check for errors and performance issues
- **Backup Configuration**: Save custom settings and configurations

### System Updates
- **Version Control**: Use Git for tracking changes
- **Testing**: Test updates in development environment first
- **Rollback Plan**: Maintain ability to revert to previous versions

## License & Attribution

This project is developed for educational and research purposes. Please ensure compliance with all applicable regulations and licensing requirements for satellite tracking and LiDAR technology.

---

**Version**: 2.0.0  
**Last Updated**: January 2024  
**Maintainer**: Drone Tracker Team  
**Contact**: For technical support and questions, please refer to the project documentation.
