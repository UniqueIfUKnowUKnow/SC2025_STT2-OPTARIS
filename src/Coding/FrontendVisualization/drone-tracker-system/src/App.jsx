// src/App.jsx (Reverted to Start Screen version)
import React, { useState, useRef, useEffect } from 'react';
import { Canvas } from '@react-three/fiber';
import Scene from './components/Scene';
import HUD from './components/HUD';
import useDroneStore from './useDroneStore';
import './App.css';

const WEBSOCKET_URL = 'ws://192.168.55.180:5678'; // <-- IMPORTANT: Use your server's IP

function App() {
  const updateData = useDroneStore((state) => state.updateData);
  const [connectionStatus, setConnectionStatus] = useState('DISCONNECTED');
  const ws = useRef(null);

  useEffect(() => {
    return () => {
      if (ws.current) {
        ws.current.close();
      }
    };
  }, []);

  const handleStart = () => {
    if (ws.current && ws.current.readyState === WebSocket.OPEN) return;

    console.log(`Connecting to ${WEBSOCKET_URL}`);
    setConnectionStatus('CONNECTING');
    
    ws.current = new WebSocket(WEBSOCKET_URL);

    ws.current.onopen = () => {
      console.log('WebSocket connection established.');
      setConnectionStatus('CONNECTED');
      console.log("Sending 'start' command to backend.");
      ws.current.send(JSON.stringify({ command: 'start' }));
    };

    ws.current.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        updateData(data);
      } catch (error) {
        console.error('Failed to parse incoming message:', error);
      }
    };

    ws.current.onerror = (error) => {
      console.error('WebSocket error:', error);
      setConnectionStatus('ERROR');
    };

    ws.current.onclose = () => {
      console.log('WebSocket connection closed.');
      setConnectionStatus('DISCONNECTED');
    };
  };

  if (connectionStatus !== 'CONNECTED') {
    return (
      <div className="start-container">
        <h1>Drone Tracker Digital Twin</h1>
        <p>System is on standby. Click start to connect and begin tracking.</p>
        <button onClick={handleStart} disabled={connectionStatus === 'CONNECTING'}>
          {connectionStatus === 'CONNECTING' ? 'Connecting...' : 'Start System'}
        </button>
        {connectionStatus === 'ERROR' && <p className="error-text">Connection Failed. Is the backend server running?</p>}
      </div>
    );
  }

  return (
    <>
      <HUD />
      <Canvas camera={{ position: [15, 10, 15], fov: 50 }}>
        <Scene />
      </Canvas>
    </>
  );
}

export default App;