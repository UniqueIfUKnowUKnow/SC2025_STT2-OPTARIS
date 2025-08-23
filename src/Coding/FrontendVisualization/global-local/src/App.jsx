import React, { useState, useEffect, useRef, useCallback, Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import * as satellite from 'satellite.js';
import { format } from 'date-fns';
import './App.css';

// Import our custom components that handle different parts of the application
import GlobalView from './components/GlobalView';
import LocalView from './components/LocalView';
import SystemHUD from './components/SystemHUD';
import StartScreen from './components/StartScreen';
import LoadingScene from './components/LoadingScene';
import ErrorBoundary from './components/ErrorBoundary';
import useUnifiedStore from './store/useUnifiedStore';
import SensorWebSocket from './services/SensorWebSocket';

// Sofia Tech Park, Bulgaria coordinates - this is where our ground station is located
// These are real GPS coordinates that we use to place the ground station marker on the 3D globe
const SOFIA_TECH_PARK = {
  lat: 42.6501,  // Latitude (north-south position on Earth)
  lon: 23.3795,  // Longitude (east-west position on Earth)
  altKm: 0.6     // Altitude in kilometers above sea level
};

// TLE Data for satellites - TLE stands for "Two-Line Element" which describes satellite orbits
// This is like a "recipe" that tells us where satellites are and how they move around Earth
const SATELLITE_TLES = {
  'Optaris': {
    name: 'Optaris',
    // These two lines contain all the orbital information for the Optaris satellite
    // They're updated regularly by space agencies to keep track of satellite positions
    tle1: "1 25544U 98067A   24001.00000000  .00002182  00000-0  40768-4 0  9990",
    tle2: "2 25544  51.6461 339.2971 0002829  68.7676 291.3964 15.48919103123456"
  },
  'ISS': {
    name: 'International Space Station',
    // Similar TLE data for the International Space Station
    tle1: "1 25544U 98067A   24001.00000000  .00002182  00000-0  40768-4 0  9990",
    tle2: "2 25544  51.6461 339.2971 0002829  68.7676 291.3964 15.48919103123456"
  }
};

function App() {
  // State variables that control what the user sees and how the app behaves
  
  // This determines whether we show the start screen or the main application
  const [showStartScreen, setShowStartScreen] = useState(true);
  
  // This tracks which view the user is currently looking at - either 'global' (Earth view) or 'local' (LiDAR view)
  const [currentView, setCurrentView] = useState('global');
  
  // This tracks whether the 3D scene is still loading - prevents white screen crashes
  const [isSceneLoading, setIsSceneLoading] = useState(false);
  
  // This tracks the current date and time for display
  const [currentDateTime, setCurrentDateTime] = useState(new Date());
  
  // This stores the WebSocket connection URL - it's how our frontend talks to the Raspberry Pi backend
  const [wsUrl, setWsUrl] = useState('ws://localhost:8765');
  
  // This tracks whether we're connected to the backend - 'connected', 'connecting', or 'disconnected'
  const [wsStatus, setWsStatus] = useState('disconnected');
  
  // This is a reference to our WebSocket connection object - we need this to send commands to the backend
  const wsRef = useRef(null);
  
  // This gets all the data and functions from our central data store (Zustand store)
  // Think of this like a "brain" that remembers all the information about satellites, positions, etc.
  const {
    status,                    // Current system status (e.g., "TRACKING_TARGET", "SEARCHING")
    progress,                  // Progress percentage for operations (0-100%)
    warning,                   // Any warning messages from the system
    error,                     // Any error messages from the system
    measuredPosition,          // The current position measured by the LiDAR sensor
    scannerPosition,           // Where the LiDAR scanner beam is currently pointing
    liveTelemetry,            // Real-time data from the sensors (angles, signal strength, etc.)
    positionHistory,          // List of previous positions (for showing movement history)
    predictedOrbitParams,     // Calculated orbital parameters (like orbit shape, size, etc.)
    satellites,               // Information about satellites we're tracking
    updateData,               // Function to update our data when we receive new information
    clearForReset,            // Function to clear all data when resetting the system
    setStatus,                // Function to update the system status
    setProgress,              // Function to update the progress percentage
    setWarning,               // Function to set warning messages
    setError                  // Function to set error messages
  } = useUnifiedStore();

  // This effect runs once when the app starts up
  // It initializes the satellite data by converting the TLE text into usable satellite objects
  useEffect(() => {
    // Create an empty object to store our satellite information
    const satObjects = {};
    
    // Go through each satellite in our TLE data
    Object.entries(SATELLITE_TLES).forEach(([id, satData]) => {
      try {
        // Convert the TLE text into a satellite object that we can use for calculations
        const rec = satellite.twoline2satrec(satData.tle1, satData.tle2);
        
        // Store the satellite information with some additional properties
        satObjects[id] = {
          ...satData,                    // Keep all the original TLE data
          rec,                           // Add the calculated satellite record
          position: null,                // Position will be calculated in real-time
          color: id === 'Optaris' ? '#ff6b35' : '#00ff88'  // Different colors for different satellites
        };
      } catch (error) {
        // If there's an error processing the satellite data, log it so we can debug
        console.error(`Error initializing satellite ${id}:`, error);
      }
    });
    
    // Store all the satellite information in our central data store
    useUnifiedStore.getState().setSatellites(satObjects);
  }, []); // Empty array means this only runs once when the app starts

  // This effect runs continuously to update the current date and time
  // It updates every second so the display is always current
  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentDateTime(new Date());
    }, 1000);

    // Clean up the timer when the component unmounts
    return () => clearInterval(timer);
  }, []);

  // This effect manages the WebSocket connection to the Raspberry Pi backend
  // It runs once when the component mounts and when wsUrl changes
  useEffect(() => {
    console.log('App: Setting up WebSocket connection to:', wsUrl);
    
    // If we already have a WebSocket connection, stop it first
    if (wsRef.current) {
      console.log('App: Stopping existing WebSocket connection');
      wsRef.current.stop();
      wsRef.current = null;
    }
    
    // Create a new WebSocket client that will connect to our backend
    const client = new SensorWebSocket({
      url: wsUrl,                        // The URL to connect to (e.g., ws://192.168.1.100:8765)
      onStatusChange: (status) => {
        console.log('App: WebSocket status changed to:', status);
        setWsStatus(status);
      },
      onMessage: (msg) => {
        console.log('App: Received WebSocket message:', msg);
        // When we receive data from the backend, update our central data store
        updateData(msg);
      },
    });
    
    // Store the WebSocket client reference so we can use it later
    wsRef.current = client;
    
    // Start the connection
    console.log('App: Starting WebSocket connection');
    client.start();
    
    // Cleanup function - this runs when the component unmounts or when the URL changes
    return () => {
      console.log('App: Cleaning up WebSocket connection');
      if (client) {
        client.stop();
      }
    };
  }, [wsUrl]); // This effect runs whenever wsUrl changes

  // Function to handle when the user clicks the start button on the start screen
  const handleStartApp = () => {
    console.log('App: handleStartApp called');
    setShowStartScreen(false);  // Hide the start screen
    setIsSceneLoading(true);    // Show loading scene to prevent white screen crash
    setCurrentView('global');   // Show the global view (Earth) by default
    console.log('App: State set - showStartScreen: false, isSceneLoading: true');
  };

  // Function to handle when the loading scene completes
  const handleLoadingComplete = useCallback(() => {
    console.log('App: handleLoadingComplete called');
    setIsSceneLoading(false);   // Hide the loading scene
    console.log('App: State set - isSceneLoading: false');
  }, []);

  // Function to handle going back to start screen from error boundary
  const handleGoHome = () => {
    setShowStartScreen(true);   // Show the start screen again
    setIsSceneLoading(false);   // Hide any loading states
  };

  // Function to switch between global view (Earth) and local view (LiDAR tracker)
  const switchView = () => {
    setCurrentView(currentView === 'global' ? 'local' : 'global');
  };

  // Function to handle when the user clicks on the ground station marker in the global view
  // This automatically switches to the local view so they can see the LiDAR tracker
  const handleGroundStationClick = () => {
    setCurrentView('local');
  };

  // Function to handle when the user clicks the Start button in the control panel
  // This sends a 'start' command to the Raspberry Pi backend
  const handleStart = () => {
    console.log('App: Start button clicked, sending start command');
    if (wsRef.current) {
      const command = { command: 'start' };
      console.log('App: Sending command:', command);
      wsRef.current.send(command);
    } else {
      console.warn('App: No WebSocket connection available');
    }
  };

  // Function to handle when the user clicks the Stop button in the control panel
  // This sends a 'stop' command to the Raspberry Pi backend
  const handleStop = () => {
    console.log('App: Stop button clicked, sending stop command');
    if (wsRef.current) {
      const command = { command: 'stop' };
      console.log('App: Sending command:', command);
      wsRef.current.send(command);
    } else {
      console.warn('App: No WebSocket connection available');
    }
  };

  // Function to handle when the user clicks the Reset button in the control panel
  // This clears all data and sends a 'reset' command to the Raspberry Pi backend
  const handleReset = () => {
    console.log('App: Reset button clicked, sending reset command');
    clearForReset();  // Clear all data in our frontend
    if (wsRef.current) {
      const command = { command: 'reset' };
      console.log('App: Sending command:', command);
      wsRef.current.send(command);  // Tell the backend to reset too
    } else {
      console.warn('App: No WebSocket connection available');
    }
  };

  // Function to handle when the user changes the WebSocket URL in the input field
  const handleWsUrlChange = (newUrl) => {
    setWsUrl(newUrl);
  };

  // Function to handle when the user clicks the Connect button
  // This creates a new WebSocket connection with the new URL
  const handleReconnect = () => {
    // Stop the current connection if it exists
    if (wsRef.current) {
      try { 
        wsRef.current.stop(); 
      } catch {} // Ignore any errors during shutdown
    }
    
    // Create a new WebSocket client with the updated URL
    const client = new SensorWebSocket({
      url: wsUrl,
      onStatusChange: setWsStatus,
      onMessage: (msg) => updateData(msg),
    });
    
    // Store the new client reference
    wsRef.current = client;
    
    // Start the new connection
    client.start();
  };

  // If we're showing the start screen, render just that
  if (showStartScreen) {
    console.log('App: Rendering StartScreen');
    return <StartScreen onStart={handleStartApp} />;
  }

  // If we're loading the 3D scene, show the loading screen
  if (isSceneLoading) {
    console.log('App: Rendering LoadingScene');
    return <LoadingScene onComplete={handleLoadingComplete} />;
  }

  console.log('App: Rendering main application');
  // Main application interface - this is what users see after clicking Start on the start screen
  return (
    <ErrorBoundary onGoHome={handleGoHome}>
      <div className="unified-tracker">
        {/* Main content area that contains the 3D scene and UI panels */}
        <div className="main-content">
          
          {/* 3D Scene Container - this is where the Earth, satellites, and LiDAR tracker are rendered */}
          <div className="scene-container">
            {/* Canvas is the 3D rendering area - it's like a window into a 3D world */}
            <Canvas
              // Camera settings - this determines the initial view and how users can move around
              camera={{ 
                // For global view: start far away to see the whole Earth
                // For local view: start closer to see the LiDAR tracker details
                position: currentView === 'global' ? [20, 20, 20] : [0, 10, 20], 
                fov: currentView === 'global' ? 75 : 50  // Field of view - how "wide" the view is
              }}
              style={{ background: '#000000' }}  // Black background like space
              gl={{ antialias: true, alpha: false }}  // Graphics settings for smooth rendering
            >
              
              {/* Render different 3D scenes based on which view the user has selected */}
              <Suspense fallback={null}>
                {currentView === 'global' ? (
                  // Global View: Shows the Earth with satellites and ground station
                  <GlobalView 
                    satellites={satellites}                    // Satellite data for rendering
                    measuredPosition={measuredPosition}        // Current LiDAR measurement (hidden in global view)
                    positionHistory={positionHistory}          // History of measurements (hidden in global view)
                    predictedOrbitParams={predictedOrbitParams} // Calculated orbit for Optaris
                    status={status}                           // Current system status
                    onGroundStationClick={handleGroundStationClick} // Function to call when ground station is clicked
                  />
                ) : (
                  // Local View: Shows the LiDAR tracker with scanner beam and target
                  <LocalView 
                    measuredPosition={measuredPosition}        // Current target position
                    scannerPosition={scannerPosition}          // Where the scanner beam is pointing
                    positionHistory={positionHistory}          // History of target positions
                    liveTelemetry={liveTelemetry}             // Real-time sensor data
                    status={status}                           // Current system status
                    predictedOrbitParams={predictedOrbitParams} // Predicted orbit to show in local view
                  />
                )}
              </Suspense>
              
              {/* OrbitControls allows users to move the camera around the 3D scene */}
              {/* Users can zoom in/out, rotate the view, and pan around */}
              <OrbitControls 
                enablePan={true}      // Allow panning (moving the view left/right/up/down)
                enableZoom={true}     // Allow zooming in/out
                enableRotate={true}   // Allow rotating the view around
                maxDistance={currentView === 'global' ? 50 : 30}  // How far users can zoom out
                minDistance={currentView === 'global' ? 5 : 2}   // How close users can zoom in
                autoRotate={false}    // Don't automatically rotate the view
              />
            </Canvas>
          </div>

          {/* HUD Overlay - this contains all the control panels and information displays */}
          {/* These panels are positioned on the sides and don't move when users interact with the 3D scene */}
          <SystemHUD
            currentView={currentView}                    // Which view we're currently showing
            status={status}                              // Current system status
            progress={progress}                          // Progress percentage
            warning={warning}                            // Warning messages
            error={error}                                // Error messages
            measuredPosition={measuredPosition}          // Current target position
            predictedOrbitParams={predictedOrbitParams} // Calculated orbital parameters
            liveTelemetry={liveTelemetry}               // Real-time sensor data
            wsStatus={wsStatus}                         // WebSocket connection status
            wsUrl={wsUrl}                               // Current WebSocket URL
            onWsUrlChange={handleWsUrlChange}           // Function to call when URL changes
            onReconnect={handleReconnect}               // Function to call when Connect is clicked
            onSwitchView={switchView}                   // Function to call when switching views
            onStart={handleStart}                       // Function to call when Start is clicked
            onStop={handleStop}                         // Function to call when Stop is clicked
            onReset={handleReset}                       // Function to call when Reset is clicked
          />
        </div>
      </div>
      
      {/* Date and Time Display - Fixed at the bottom of the screen */}
      <div className="datetime-overlay">
        <div className="date-display">
          {format(currentDateTime, 'EEEE, MMMM do, yyyy')}
        </div>
        <div className="time-display">
          {format(currentDateTime, 'HH:mm:ss')}
        </div>
      </div>
    </ErrorBoundary>
  );
}

// Export the App component so it can be used by the main application
export default App;
