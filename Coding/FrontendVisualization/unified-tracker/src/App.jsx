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
  
  // NEW: This tracks the current application mode (simulation or real)
  const [appMode, setAppMode] = useState('simulation');
  
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
  // It runs once when the component mounts and when wsUrl or appMode changes
  useEffect(() => {
    // NEW: Only establish WebSocket connection in Real Mode
    if (appMode !== 'real') {
      console.log('App: Simulation mode - no WebSocket connection needed');
      // Clear any existing WebSocket connection
      if (wsRef.current) {
        wsRef.current.stop();
        wsRef.current = null;
      }
      setWsStatus('disconnected');
      return;
    }
    
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
        
        // Clear any previous warnings/errors when connection status changes
        if (status === 'connected') {
          setWarning(null);
          setError(null);
        } else if (status === 'error') {
          setError(`Failed to connect to ${wsUrl}. Check if the server is running and accessible.`);
        }
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
  }, [wsUrl, appMode]); // This effect runs whenever wsUrl or appMode changes

  // Function to handle when the user clicks the start button on the start screen
  // Modified to handle simple start without mode parameters
  const handleStartApp = () => {
    console.log('App: Starting application');
    setAppMode('simulation'); // Default to simulation mode
    setWsUrl('ws://localhost:8765'); // Default WebSocket URL
    setShowStartScreen(false);  // Hide the start screen
    setIsSceneLoading(true);    // Show loading scene to prevent white screen crash
    setCurrentView('global');   // Show the global view (Earth) by default
    console.log('App: State set - showStartScreen: false, isSceneLoading: true, appMode: simulation');
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
    console.log('App: Ground station clicked, switching to local view');
    setCurrentView('local');
  };

  // Function to handle when the user clicks the Start button in the SystemHUD
  // This starts the tracking process
  const handleStart = () => {
    console.log('App: handleStart called');
    
    // NEW: In Real Mode, we can't manually start - it's controlled by the Raspberry Pi
    if (appMode === 'real') {
      console.log('App: Real Mode - Start button disabled, process controlled by Raspberry Pi');
      setWarning('In Real Mode, the scanning process is controlled by the Raspberry Pi. Use the backend script to start operations.');
      return;
    }
    
    // In Simulation Mode, start the simulation
    if (wsRef.current) {
      console.log('App: Sending start command via WebSocket');
      wsRef.current.send({ command: 'start' });
    } else {
      console.warn('App: No WebSocket connection available');
      // Fallback: start simulation locally
      setStatus('SEARCHING_FOR_TARGET');
      setProgress(0);
    }
  };

  // Function to handle when the user clicks the Stop button in the SystemHUD
  // This stops the tracking process
  const handleStop = () => {
    console.log('App: handleStop called');
    
    // NEW: In Real Mode, we can't manually stop - it's controlled by the Raspberry Pi
    if (appMode === 'real') {
      console.log('App: Real Mode - Stop button disabled, process controlled by Raspberry Pi');
      setWarning('In Real Mode, the scanning process is controlled by the Raspberry Pi. Use the backend script to stop operations.');
      return;
    }
    
    if (wsRef.current) {
      console.log('App: Sending stop command via WebSocket');
      wsRef.current.send({ command: 'stop' });
    } else {
      console.warn('App: No WebSocket connection available');
      // Fallback: stop simulation locally
      setStatus('STANDBY');
      setProgress(0);
    }
  };

  // Function to handle when the user clicks the Reset button in the SystemHUD
  // This resets all data and returns to standby
  const handleReset = () => {
    console.log('App: handleReset called');
    
    // NEW: In Real Mode, we can't manually reset - it's controlled by the Raspberry Pi
    if (appMode === 'real') {
      console.log('App: Real Mode - Reset button disabled, process controlled by Raspberry Pi');
      setWarning('In Real Mode, the scanning process is controlled by the Raspberry Pi. Use the backend script to reset operations.');
      return;
    }
    
    if (wsRef.current) {
      console.log('App: Sending reset command via WebSocket');
      wsRef.current.send({ command: 'reset' });
    } else {
      console.warn('App: No WebSocket connection available');
      // Fallback: reset locally
      clearForReset();
    }
  };

  // Function to handle when the user changes the WebSocket URL in the input field
  // This creates a new WebSocket connection with the new URL
  const handleWsUrlChange = (newUrl) => {
    console.log('App: WebSocket URL changed to:', newUrl);
    setWsUrl(newUrl);
    
    // NEW: Only reconnect if we're in Real Mode
    if (appMode !== 'real') {
      console.log('App: Simulation mode - no WebSocket reconnection needed');
      return;
    }
    
    // If we have an existing WebSocket connection, stop it
    if (wsRef.current) {
      console.log('App: Stopping existing WebSocket connection for URL change');
      wsRef.current.stop();
      wsRef.current = null;
    }
    
    // Create a new WebSocket client with the updated URL
    const client = new SensorWebSocket({
      url: newUrl, // FIXED: Use newUrl instead of wsUrl
      onStatusChange: (status) => {
        console.log('App: New WebSocket status changed to:', status);
        setWsStatus(status);
      },
      onMessage: (msg) => {
        console.log('App: Received message from new WebSocket:', msg);
        updateData(msg);
      },
    });
    
    // Store the new WebSocket client reference
    wsRef.current = client;
    
    // Start the new connection
    console.log('App: Starting new WebSocket connection with updated URL');
    client.start();
  };

  // Function to handle when the user clicks the Connect button in the SystemHUD
  // This attempts to reconnect to the WebSocket server
  const handleReconnect = () => {
    console.log('App: handleReconnect called');
    
    // NEW: Only reconnect if we're in Real Mode
    if (appMode !== 'real') {
      console.log('App: Simulation mode - no WebSocket reconnection needed');
      setWarning('Please switch to Real Mode to connect to WebSocket');
      return;
    }
    
    console.log('App: Attempting to connect to WebSocket at:', wsUrl);
    
    if (wsRef.current) {
      console.log('App: Stopping existing WebSocket connection for reconnection');
      wsRef.current.stop();
      wsRef.current = null;
    }
    
    // Create a new WebSocket client
    const client = new SensorWebSocket({
      url: wsUrl, // This is correct here since we're reconnecting to the current URL
      onStatusChange: (status) => {
        console.log('App: Reconnected WebSocket status changed to:', status);
        setWsStatus(status);
        
        // Clear any previous warnings/errors when connection status changes
        if (status === 'connected') {
          setWarning(null);
          setError(null);
        } else if (status === 'error') {
          setError(`Failed to connect to ${wsUrl}. Check if the server is running and accessible.`);
        }
      },
      onMessage: (msg) => {
        console.log('App: Received message from reconnected WebSocket:', msg);
        updateData(msg);
      },
    });
    
    // Store the new WebSocket client reference
    wsRef.current = client;
    
    // Start the new connection
    console.log('App: Starting reconnected WebSocket connection');
    client.start();
  };

  // NEW: Function to handle mode changes from SystemHUD
  const handleModeChange = (newMode) => {
    console.log('App: Mode changed to:', newMode);
    setAppMode(newMode);
    
    // If switching to simulation mode, disconnect WebSocket
    if (newMode === 'simulation') {
      if (wsRef.current) {
        console.log('App: Switching to simulation mode - stopping WebSocket');
        wsRef.current.stop();
        wsRef.current = null;
      }
      setWsStatus('disconnected');
      setWarning(null);
      setError(null);
    }
    // If switching to real mode, establish WebSocket connection
    else if (newMode === 'real') {
      console.log('App: Switching to real mode - establishing WebSocket connection');
      // The useEffect will handle the WebSocket connection
    }
  };

  // NEW: Function to test WebSocket connection
  const testWebSocketConnection = () => {
    console.log('App: Testing WebSocket connection to:', wsUrl);
    
    if (appMode !== 'real') {
      setWarning('Please switch to Real Mode to test WebSocket connection');
      return;
    }
    
    // Create a temporary WebSocket to test the connection
    const testSocket = new WebSocket(wsUrl);
    
    testSocket.onopen = () => {
      console.log('App: Test connection successful to:', wsUrl);
      setWarning(`Test connection successful to ${wsUrl}`);
      testSocket.close();
    };
    
    testSocket.onerror = (error) => {
      console.error('App: Test connection failed to:', wsUrl, error);
      setError(`Test connection failed to ${wsUrl}. Check if the server is running.`);
    };
    
    testSocket.onclose = () => {
      console.log('App: Test connection closed');
    };
    
    // Set a timeout to close the test connection if it doesn't connect quickly
    setTimeout(() => {
      if (testSocket.readyState === WebSocket.CONNECTING) {
        testSocket.close();
        setError(`Test connection timeout to ${wsUrl}. Server may be unreachable.`);
      }
    }, 5000);
  };

  // If we're showing the start screen, render it
  if (showStartScreen) {
    console.log('App: Rendering start screen');
    return (
      <StartScreen onStart={handleStartApp} />
    );
  }

  // If the scene is loading, show the loading scene
  if (isSceneLoading) {
    console.log('App: Rendering loading scene');
    return (
      <LoadingScene onComplete={handleLoadingComplete} />
    );
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
                    progress={progress}                       // NEW: Progress percentage for real-time display
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
            appMode={appMode}                            // NEW: Current application mode
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
            onModeChange={handleModeChange}            // Function to call when mode changes
            onTestConnection={testWebSocketConnection}  // NEW: Function to test WebSocket connection
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
