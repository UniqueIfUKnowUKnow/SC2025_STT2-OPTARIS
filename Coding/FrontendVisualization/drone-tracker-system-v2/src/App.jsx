// src/App.jsx (Reverted to Start Screen version)
// This is the main application component that manages the WebSocket connection
// and renders either the start screen or the 3D visualization based on connection status

// Import React hooks for state management, refs, and side effects
import React, { useState, useRef, useEffect } from 'react';
// Import Canvas component from React Three Fiber for 3D rendering
import { Canvas } from '@react-three/fiber';
// Import our custom components
import Scene from './components/Scene';
import HUD from './components/HUD';
// Import our global state management store
import useDroneStore from './useDroneStore';
// Import CSS styles for this component
import './App.css';

// ErrorBoundary component to catch rendering errors
class ErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null };
  }
  static getDerivedStateFromError(error) {
    return { hasError: true, error };
  }
  componentDidCatch(error, errorInfo) {
    // Log error to console or send to monitoring service
    console.error('UI ErrorBoundary caught:', error, errorInfo);
  }
  render() {
    if (this.state.hasError) {
      return (
        <div className="error-fallback">
          <h2>Something went wrong.</h2>
          <pre>{this.state.error?.toString()}</pre>
          <button onClick={() => window.location.reload()}>Reload</button>
        </div>
      );
    }
    return this.props.children;
  }
}

// WebSocket server URL - IMPORTANT: Change this to your backend server's IP address
const WEBSOCKET_URL = 'ws://192.168.55.180:5678'; // <-- IMPORTANT: Use your server's IP

// Main App component that orchestrates the entire application
function App() {
  // Get store actions to update data and clear state on reset
  const updateData = useDroneStore((state) => state.updateData);
  const clearForReset = useDroneStore((state) => state.clearForReset);
  
  // Local state to track WebSocket connection status
  const [connectionStatus, setConnectionStatus] = useState('DISCONNECTED');
  
  // Use ref to store WebSocket instance (persists across re-renders)
  const ws = useRef(null);
  // Use ref to flag whether messages should be processed (used for Stop/Resume)
  const isPaused = useRef(false);

  // Cleanup effect: close WebSocket connection when component unmounts
  useEffect(() => {
    // Return cleanup function that runs when component unmounts
    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []); // Empty dependency array means this runs only on mount/unmount

  // Helper to create or reuse an open WebSocket connection
  const ensureConnection = () => {
    // If there's already an open connection, return it
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      return ws.current;
    }
    // If a connection exists but is not open, close and recreate
    if (ws.current && ws.current.readyState !== WebSocket.CLOSED) {
      try { ws.current.close(); } catch {}
    }

    // Create a new WebSocket connection
    const socket = new WebSocket(WEBSOCKET_URL);
    ws.current = socket;

    // Set temporary status while connecting
    setConnectionStatus('CONNECTING');

    // Handle successful connection
    socket.onopen = () => {
      // Mark as connected
      setConnectionStatus('CONNECTED');
      // Send start command to backend to begin or resume simulation
      socket.send(JSON.stringify({ command: 'start' }));
    };

    // Handle incoming messages from the backend
    socket.onmessage = (event) => {
      // If paused, ignore messages to effectively "stop" updates without closing socket
      if (isPaused.current) return;
      try {
        // Parse the JSON data from the WebSocket message
        const data = JSON.parse(event.data);
        // Update our global store with the new data
        updateData(data);
      } catch (error) {
        // Log any errors that occur while parsing the message
        console.error('Failed to parse incoming message:', error, event.data);
        setConnectionStatus('ERROR');
      }
    };

    // Handle WebSocket errors
    socket.onerror = (error) => {
      console.error('WebSocket error:', error);
      setConnectionStatus('ERROR');
    };

    // Handle WebSocket connection closure
    socket.onclose = () => {
      console.log('WebSocket connection closed.');
      setConnectionStatus('DISCONNECTED');
    };

    // Return the new socket for immediate use
    return socket;
  };

  // Function to handle starting or resuming the WebSocket data flow
  const handleStart = () => {
    // Clear paused flag so messages are processed
    isPaused.current = false;
    // Ensure we have an open connection
    const socket = ensureConnection();
    // If already open, send a start command to resume on backend side
    if (socket && socket.readyState === WebSocket.OPEN) {
      try { socket.send(JSON.stringify({ command: 'start' })); } catch {}
    }
  };

  // Function to handle pausing the UI updates while keeping scene state
  const handleStop = () => {
    // Set paused flag so onmessage ignores inbound updates
    isPaused.current = true;
  };

  // Function to handle clearing UI and requesting backend reset
  const handleReset = () => {
    // Pause updates while we clear UI and ask backend to reset
    isPaused.current = true;
    // Clear UI state (history, warnings, errors, orbits) and set status to Idle
    clearForReset();
    // If socket is open, send reset command to backend
    if (ws.current && ws.current.readyState === WebSocket.OPEN) {
      try { ws.current.send(JSON.stringify({ command: 'reset' })); } catch {}
    }
    // Optionally reconnect fresh to restart from beginning
    try { if (ws.current) ws.current.close(); } catch {}
    ws.current = null;
    setConnectionStatus('DISCONNECTED');
  };

  // Render start screen if not connected
  if (connectionStatus !== 'CONNECTED') {
    return (
      <ErrorBoundary>
        <div className="start-container">
          {/* Application title */}
          <h1>Drone Tracker Digital Twin</h1>
          {/* Instructions for the user */}
          <p>System is on standby. Click start to connect and begin tracking.</p>
          {/* Start button - disabled while connecting to prevent multiple clicks */}
          <button onClick={handleStart} disabled={connectionStatus === 'CONNECTING'}>
            {connectionStatus === 'CONNECTING' ? 'Connecting...' : 'Start System'}
          </button>
          {/* Show error message if connection failed */}
          {connectionStatus === 'ERROR' && <p className="error-text">Connection Failed. Is the backend server running?</p>}
        </div>
      </ErrorBoundary>
    );
  }

  // Render the main 3D visualization when connected
  return (
    <ErrorBoundary>
      <div style={{ width: '100vw', height: '100vh', position: 'relative', overflow: 'hidden' }}>
        {/* Heads-up display overlay with controls and telemetry */}
        <HUD handleStart={handleStart} handleStop={handleStop} handleReset={handleReset} />
        {/* 3D Canvas with camera positioned for good viewing angle */}
        <Canvas style={{ width: '100vw', height: '100vh', display: 'block' }} camera={{ position: [0, 10, 20], fov: 50 }}>
          {/* Render the 3D scene with all drone visualization components */}
          <Scene />
        </Canvas>
      </div>
    </ErrorBoundary>
  );
}

// Export the App component as the default export
export default App;