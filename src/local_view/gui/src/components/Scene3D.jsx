import React, { useMemo, useState } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, Grid } from '@react-three/drei'
import * as THREE from 'three'
// Imports a custom hook to handle the WebSocket connection and data
import useWebSocket from '../hooks/useWebSocket.jsx'

// Renders a simple red cube at the origin, representing the lidar sensor.
const Box = () => {
  return (
    <mesh position={[0, 0, 0]}>
      <boxGeometry args={[1, 1, 1]} />
      <meshStandardMaterial color="red" />
    </mesh>
  )
}

// Renders a transparent sphere with a 12-meter radius, visually representing the lidar's range.
const Sphere = () => {
  return (
    <mesh position={[0, 0, 0]}>
      <sphereGeometry args={[12, 32, 32]} />
      <meshStandardMaterial color="blue" transparent opacity={0.2} />
    </mesh>
  )
}

// Renders the X, Y, and Z axes using thin colored boxes.
const Axes = () => {
  return (
    <group>
      <mesh position={[5, 0, 0]}>
        <boxGeometry args={[10, 0.1, 0.1]} />
        <meshStandardMaterial color="red" /> // X-axis (red)
      </mesh>
      <mesh position={[0, 5, 0]}>
        <boxGeometry args={[0.1, 10, 0.1]} />
        <meshStandardMaterial color="green" /> // Y-axis (green)
      </mesh>
      <mesh position={[0, 0, 5]}>
        <boxGeometry args={[0.1, 0.1, 10]} />
        <meshStandardMaterial color="blue" /> // Z-axis (blue)
      </mesh>
    </group>
  )
}

// Renders a collection of points received from the WebSocket.
const Points = ({ points }) => {
  // Use useMemo to prevent re-creating the geometry on every render unless the points data changes.
  const pointsGeometry = useMemo(() => {
    const geometry = new THREE.BufferGeometry();
  
    if (points.length > 0) {
      // Create a flattened array of position data (x, y, z, x, y, z...).
      const positions = new Float32Array(points.length * 3);
      points.forEach((point, i) => {
        positions[i * 3] = point.x;
        positions[i * 3 + 1] = point.y;
        positions[i * 3 + 2] = point.z;
      });
      
      // Set the position attribute for the geometry. This is how Three.js knows where to draw the points.
      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    }
    
    return geometry;
  }, [points]); // The dependency array ensures this memoization is updated when new points are received.

  return (
    <points>
      <primitive object={pointsGeometry} />
      <pointsMaterial
        size={0.2}
        color="white"
        sizeAttenuation={true}
        transparent={true}
        opacity={0.8}
      />
    </points>
  );
};

// Component to display the WebSocket connection status to the user.
const ConnectionStatus = ({ isConnected, error }) => {
  // Determines the background color and text based on connection state or error.
  const backgroundColor = error ? '#ff4444' : (isConnected ? '#44ff44' : '#ffff44');
  const text = error ? 'Error: ' + error : (isConnected ? 'Connected' : 'Connecting...');

  return (
    <div style={{
      position: 'absolute',
      top: '10px',
      right: '10px',
      padding: '8px 12px',
      backgroundColor,
      color: 'black',
      borderRadius: '4px',
      fontFamily: 'monospace',
      zIndex: 1000
    }}>
      {text}
    </div>
  );
};

// Main component that orchestrates the entire scene and UI.
const Scene3D = () => {
  // State hook to manage the WebSocket URL input value.
  const [wsUrl, setWsUrl] = useState('');
  // The useWebSocket hook provides the live data stream and connection state.
  const { points, isConnected, error, connect, disconnect, clearPoints } = useWebSocket();

  // Function to handle the connect button click event.
  const handleConnect = () => {
    if (wsUrl) {
      connect(wsUrl);
    }
  };

  return (
    <div style={{ width: '100vw', height: '100vh', backgroundColor: '#000000', position: 'relative' }}>
      {/* Displays the connection status indicator. */}
      <ConnectionStatus isConnected={isConnected} error={error} />
      {/* UI container for the input field and control buttons. */}
      <div style={{
        position: 'absolute',
        top: '10px',
        left: '10px',
        display: 'flex',
        gap: '10px',
        zIndex: 1000
      }}>
        {/* Input field for the WebSocket URL. */}
        <input
          type="text"
          value={wsUrl}
          onChange={(e) => setWsUrl(e.target.value)}
          placeholder="ws://your.target.ip.address:nnnn"
          style={{
            padding: '8px',
            borderRadius: '4px',
            border: '1px solid #666',
            backgroundColor: '#333',
            color: 'white',
            width: '200px'
          }}
        />
        {/* Button to connect or disconnect from the WebSocket. */}
        <button
          onClick={isConnected ? disconnect : handleConnect}
          style={{
            padding: '8px 12px',
            backgroundColor: isConnected ? '#ff4444' : '#44ff44',
            color: 'black',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer'
          }}
        >
          {isConnected ? 'Disconnect' : 'Connect'}
        </button>
        {/* Button to clear all visualized points. */}
        <button
          onClick={clearPoints}
          style={{
            padding: '8px 12px',
            backgroundColor: '#666',
            color: 'white',
            border: 'none',
            borderRadius: '4px',
            cursor: 'pointer'
          }}
        >
          Clear Points
        </button>
      </div>
      {/* Main canvas for the 3D scene, with camera settings. */}
      <Canvas camera={{ position: [15, 15, 15], fov: 75 }}>
        <color attach="background" args={['#1a1a1a']} />
        {/* Renders a grid to provide a reference frame for the scene. */}
        <Grid
          args={[30, 30]}
          cellSize={1}
          cellThickness={0.5}
          cellColor="#505050"
          sectionSize={5}
          sectionThickness={1}
          sectionColor="#808080"
          fadeDistance={30}
          fadeStrength={1}
          followCamera={false}
          infiniteGrid={true}
        />
        {/* Renders the visual elements in the 3D space. */}
        <Box />
        <Sphere />
        <Axes />
        {/* Renders the dynamic points from the WebSocket data stream. */}
        <Points points={points} />
        {/* Adds ambient and point lighting to the scene. */}
        <ambientLight intensity={0.4} />
        <pointLight position={[10, 10, 10]} intensity={0.6} />
        {/* Allows user to control the camera with mouse interactions. */}
        <OrbitControls maxDistance={50} minDistance={5} />
      </Canvas>
    </div>
  )
}

export default Scene3D