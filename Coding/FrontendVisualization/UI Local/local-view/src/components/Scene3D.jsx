import React, { useMemo, useState } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls, Grid } from '@react-three/drei'
import * as THREE from 'three'
import useWebSocket from '../hooks/useWebSocket.jsx'

const Box = () => {
  return (
    <mesh position={[0, 0, 0]}>
      <boxGeometry args={[1, 1, 1]} />
      <meshStandardMaterial color="red" />
    </mesh>
  )
}

const Sphere = () => {
  return (
    <mesh position={[0, 0, 0]}>
      <sphereGeometry args={[12, 32, 32]} />
      <meshStandardMaterial color="blue" transparent opacity={0.2} />
    </mesh>
  )
}

const Axes = () => {
  return (
    <group>
      <mesh position={[5, 0, 0]}>
        <boxGeometry args={[10, 0.1, 0.1]} />
        <meshStandardMaterial color="red" />
      </mesh>
      <mesh position={[0, 5, 0]}>
        <boxGeometry args={[0.1, 10, 0.1]} />
        <meshStandardMaterial color="green" />
      </mesh>
      <mesh position={[0, 0, 5]}>
        <boxGeometry args={[0.1, 0.1, 10]} />
        <meshStandardMaterial color="blue" />
      </mesh>
    </group>
  )
}



const Points = ({ points }) => {
  const pointsGeometry = useMemo(() => {
    const geometry = new THREE.BufferGeometry();
  
    if (points.length > 0) {
      const positions = new Float32Array(points.length * 3);
      points.forEach((point, i) => {
        positions[i * 3] = point.x;
        positions[i * 3 + 1] = point.y;
        positions[i * 3 + 2] = point.z;
      });
      
      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    }
    
    return geometry;
  }, [points]);

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

const ConnectionStatus = ({ isConnected, error }) => {
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

const Scene3D = () => {
  const [wsUrl, setWsUrl] = useState('');
  const { points, isConnected, error, connect, disconnect, clearPoints } = useWebSocket();

  const handleConnect = () => {
    if (wsUrl) {
      connect(wsUrl);
    }
  };

  return (
    <div style={{ width: '100vw', height: '100vh', backgroundColor: '#000000', position: 'relative' }}>
      <ConnectionStatus isConnected={isConnected} error={error} />
      <div style={{
        position: 'absolute',
        top: '10px',
        left: '10px',
        display: 'flex',
        gap: '10px',
        zIndex: 1000
      }}>
        <input
          type="text"
          value={wsUrl}
          onChange={(e) => setWsUrl(e.target.value)}
          placeholder="ws://localhost:8080"
          style={{
            padding: '8px',
            borderRadius: '4px',
            border: '1px solid #666',
            backgroundColor: '#333',
            color: 'white',
            width: '200px'
          }}
        />
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
      <Canvas camera={{ position: [15, 15, 15], fov: 75 }}>
        <color attach="background" args={['#1a1a1a']} />
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
        <Box />
        <Sphere />
        <Axes />
        <Points points={points} />
        <ambientLight intensity={0.4} />
        <pointLight position={[10, 10, 10]} intensity={0.6} />
        <OrbitControls maxDistance={50} minDistance={5} />
      </Canvas>
    </div>
  )
}

export default Scene3D
