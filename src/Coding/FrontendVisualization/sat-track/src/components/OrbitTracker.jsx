import React, { useState, useEffect, useRef, useMemo, Suspense } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import * as satellite from 'satellite.js';
import { format } from 'date-fns';
import RealisticEarth from './RealisticEarth';
import SimpleEarth from './SimpleEarth';
import Starfield from './Starfield';
import './OrbitTracker.css';
import SensorWebSocket from '../services/SensorWebSocket';
import { geodeticToEcef, lidarSphericalToEcefDelta, ecefToSceneArray, EARTH_RADIUS_KM } from '../utils/geo';
import { Html } from '@react-three/drei';

// TLE Data for ISS (International Space Station) - faster orbit, ~90 minute period
const TLE_LINE1 = "1 25544U 98067A   24001.00000000  .00002182  00000-0  40768-4 0  9990";
const TLE_LINE2 = "2 25544  51.6461 339.2971 0002829  68.7676 291.3964 15.48919103123456";

// Earth constants
const EARTH_RADIUS_KM_LOCAL = EARTH_RADIUS_KM; // Reuse from geo utils
const SCALE_FACTOR = 0.001; // Scale factor to make visualization manageable

// Coordinate conversion utilities
const convertEciToCartesian = (eciPosition) => {
  // ECI coordinates are already in Cartesian form (x, y, z) in km
  // We just need to scale them for visualization
  return [
    eciPosition.x * SCALE_FACTOR,
    eciPosition.z * SCALE_FACTOR, // Swap Y and Z for proper orientation
    -eciPosition.y * SCALE_FACTOR
  ];
};



// Enhanced lighting component for realistic space environment
const SpaceLighting = () => {
  return (
    <>
      {/* Main sun light - enhanced for better normal map visibility */}
      <directionalLight 
        position={[100, 50, 50]} 
        intensity={4} 
        color="#ffffff" 
        castShadow
        shadow-mapSize-width={2048}
        shadow-mapSize-height={2048}
      />
      
      {/* Enhanced ambient light for better surface detail visibility */}
      <ambientLight intensity={0.2} color="#ffffff" />
      
      {/* Additional directional lights for better surface illumination */}
      <directionalLight 
        position={[-50, 30, 50]} 
        intensity={1.5} 
        color="#ffeeaa" 
      />
      
      {/* Rim light for atmospheric effect */}
      <pointLight position={[0, 0, 100]} intensity={0.5} color="#66aaff" />
    </>
  );
};

// Axes component
const Axes = () => {
  return (
    <group>
      {/* X-axis (Red) */}
      <mesh position={[50, 0, 0]}>
        <cylinderGeometry args={[0.1, 0.1, 100]} />
        <meshStandardMaterial color="red" />
      </mesh>
      {/* Y-axis (Green) */}
      <mesh position={[0, 50, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.1, 0.1, 100]} />
        <meshStandardMaterial color="green" />
      </mesh>
      {/* Z-axis (Blue) */}
      <mesh position={[0, 0, 50]} rotation={[0, Math.PI / 2, 0]}>
        <cylinderGeometry args={[0.1, 0.1, 100]} />
        <meshStandardMaterial color="blue" />
      </mesh>
    </group>
  );
};

// Satellite marker component with pulsing animation
const SatelliteMarker = ({ position, color, size = 0.8 }) => {
  const meshRef = useRef();
  
  useEffect(() => {
    if (!meshRef.current) return;
    
    const animate = () => {
      if (meshRef.current) {
        // Pulsing animation
        const time = Date.now() * 0.005;
        meshRef.current.scale.setScalar(1 + Math.sin(time) * 0.3);
      }
      requestAnimationFrame(animate);
    };
    animate();
  }, []);
  
  if (!position) return null;
  
  return (
    <mesh ref={meshRef} position={position}>
      <sphereGeometry args={[size, 16, 16]} />
      <meshStandardMaterial 
        color={color} 
        emissive={color} 
        emissiveIntensity={0.5}
        transparent
        opacity={0.9}
      />
    </mesh>
  );
};

// Complete orbit path component
const OrbitPath = ({ satelliteRec, currentTime }) => {
  const orbitPoints = useMemo(() => {
    if (!satelliteRec || !currentTime) return [];
    
    const points = [];
    const orbitPeriod = 93; // ISS orbit period in minutes
    const numPoints = 300; // More points for smoother orbit
    
    // Generate orbit path for one complete orbit
    for (let i = 0; i < numPoints; i++) {
      const timeOffset = (i / numPoints) * orbitPeriod * 60 * 1000; // Convert to milliseconds
      const futureTime = new Date(currentTime.getTime() - (orbitPeriod * 30 * 1000) + timeOffset); // Start from 30 minutes ago
      
      try {
        const positionAndVelocity = satellite.propagate(satelliteRec, futureTime);
        if (positionAndVelocity.position && !positionAndVelocity.position.error) {
           const cartesianPos = convertEciToCartesian(positionAndVelocity.position);
          points.push(new THREE.Vector3(cartesianPos[0], cartesianPos[1], cartesianPos[2]));
        }
      } catch (error) {
        console.warn('Error generating orbit point:', error);
      }
    }
    
    return points;
  }, [satelliteRec, Math.floor(currentTime?.getTime() / 60000)]); // Update every minute

  if (orbitPoints.length < 2) return null;
  
  return (
    <line>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          count={orbitPoints.length}
          array={new Float32Array(orbitPoints.flatMap(p => [p.x, p.y, p.z]))}
          itemSize={3}
        />
      </bufferGeometry>
      <lineBasicMaterial color="#00ff88" opacity={0.9} transparent linewidth={3} />
    </line>
  );
};

// Measured path component with lines from Earth surface
const MeasuredPath = ({ positions }) => {
  if (!positions || positions.length < 2) return null;
  
  const earthRadius = EARTH_RADIUS_KM_LOCAL * SCALE_FACTOR;
  const lineSegments = [];
  
  // For each satellite position, create a line from Earth surface to satellite
  positions.forEach(pos => {
    const satPosition = new THREE.Vector3(pos[0], pos[1], pos[2]);
    
    // Calculate the direction from Earth center to satellite
    const direction = satPosition.clone().normalize();
    
    // Find the point on Earth's surface along this direction
    const surfacePoint = direction.clone().multiplyScalar(earthRadius);
    
    // Add both points to create a line segment
    lineSegments.push(surfacePoint, satPosition);
  });
  
  return (
    <line>
      <bufferGeometry>
        <bufferAttribute
          attach="attributes-position"
          count={lineSegments.length}
          array={new Float32Array(lineSegments.flatMap(p => [p.x, p.y, p.z]))}
          itemSize={3}
        />
      </bufferGeometry>
      <lineBasicMaterial color="#ff0000" opacity={0.6} transparent />
    </line>
  );
};

// Multiple satellite TLEs for comprehensive tracking
const SATELLITE_TLES = {
  'ISS': {
    name: 'International Space Station',
    tle1: "1 25544U 98067A   24001.00000000  .00002182  00000-0  40768-4 0  9990",
    tle2: "2 25544  51.6461 339.2971 0002829  68.7676 291.3964 15.48919103123456"
  }
};

// Labeled hexagon marker component for satellites
const LabeledHexagonMarker = ({ position, name, color, size = 0.15 }) => {
  if (!position) return null;
  
  return (
    <group position={position}>
      {/* Main hexagon plate - flat and thin like NASA Eyes */}
      <mesh>
        <cylinderGeometry args={[size, size, size * 0.05, 6]} />
        <meshStandardMaterial 
          color={color} 
          emissive={color} 
          emissiveIntensity={0.4}
          transparent
          opacity={0.95}
          metalness={0.3}
          roughness={0.2}
        />
      </mesh>
      
      {/* Border ring for better visibility */}
      <mesh>
        <ringGeometry args={[size * 0.9, size, 6]} />
        <meshStandardMaterial 
          color="#ffffff" 
          emissive="#ffffff" 
          emissiveIntensity={0.2}
          transparent
          opacity={0.8}
        />
      </mesh>
      
      {/* Text label with better contrast */}
      <Html position={[0, size + 0.08, 0]} center>
        <div style={{
          background: 'rgba(0,0,0,0.9)',
          color: 'white',
          padding: '3px 8px',
          borderRadius: '4px',
          fontSize: '11px',
          fontWeight: 'bold',
          whiteSpace: 'nowrap',
          border: '1px solid rgba(255,255,255,0.5)',
          boxShadow: '0 2px 8px rgba(0,0,0,0.8)',
          textShadow: '1px 1px 2px rgba(0,0,0,0.8)'
        }}>
          {name}
        </div>
      </Html>
    </group>
  );
};

// Multi-satellite orbit paths component
const MultiSatelliteOrbits = ({ satellites, currentTime }) => {
  const orbitPaths = useMemo(() => {
    if (!satellites || !currentTime) return [];
    
    const paths = [];
    Object.entries(satellites).forEach(([id, sat]) => {
      if (!sat.rec) return;
      
      const points = [];
      const orbitPeriod = 90; // Approximate period in minutes
      const numPoints = 200;
      
      for (let i = 0; i < numPoints; i++) {
        const timeOffset = (i / numPoints) * orbitPeriod * 60 * 1000;
        const futureTime = new Date(currentTime.getTime() - (orbitPeriod * 30 * 1000) + timeOffset);
        
        try {
          const positionAndVelocity = satellite.propagate(sat.rec, futureTime);
          if (positionAndVelocity.position && !positionAndVelocity.position.error) {
            const cartesianPos = convertEciToCartesian(positionAndVelocity.position);
            points.push(new THREE.Vector3(cartesianPos[0], cartesianPos[1], cartesianPos[2]));
          }
        } catch (error) {
          console.warn(`Error generating orbit for ${id}:`, error);
        }
      }
      
      if (points.length > 1) {
        paths.push({ id, points, color: sat.color });
      }
    });
    
    return paths;
  }, [satellites, Math.floor(currentTime?.getTime() / 60000)]);
  
  return (
    <>
      {orbitPaths.map(({ id, points, color }) => (
        <line key={id}>
          <bufferGeometry>
            <bufferAttribute
              attach="attributes-position"
              count={points.length}
              array={new Float32Array(points.flatMap(p => [p.x, p.y, p.z]))}
              itemSize={3}
            />
          </bufferGeometry>
          <lineBasicMaterial color={color} opacity={0.6} transparent linewidth={1} />
        </line>
      ))}
    </>
  );
};

// 3D Scene component
const Scene3D = ({ 
  predictedPosition, 
  measuredPosition,
  measuredPositions, 
  satelliteRec,
  currentTime,
  manualPointsScene,
  kfPredictedPath,
  satellites
}) => {
  return (
    <div style={{ width: '80%', height: '100%' }}>
      <Canvas
        camera={{ position: [20, 20, 20], fov: 75 }}
        style={{ background: '#000000' }}
        gl={{ antialias: true, alpha: false }}
      >
        {/* Enhanced space lighting */}
        <SpaceLighting />
        
        {/* Starfield background */}
        <Starfield count={8000} radius={400} />
        
        {/* Earth with Suspense for texture loading */}
        <Suspense fallback={
          <mesh>
            <sphereGeometry args={[EARTH_RADIUS_KM * SCALE_FACTOR, 64, 32]} />
            <meshStandardMaterial color="#2E7D32" />
          </mesh>
        }>
          <SimpleEarth radius={EARTH_RADIUS_KM * SCALE_FACTOR} />
        </Suspense>
        
        {/* Multi-satellite orbital paths */}
        <MultiSatelliteOrbits satellites={satellites} currentTime={currentTime} />
        
        {/* Satellite markers with labels */}
        {Object.entries(satellites).map(([id, sat]) => (
          <LabeledHexagonMarker 
            key={id}
            position={sat.position} 
            name={sat.name}
            color={sat.color} 
            size={0.2} 
          />
        ))}
        
        {/* Current satellite position (the moving dot) */}
        <SatelliteMarker 
          position={predictedPosition} 
          color="#ff6b35" 
          size={0.5} 
        />

        {/* Measured position (LiDAR-derived) */}
        <SatelliteMarker 
          position={measuredPosition} 
          color="#4e9cff" 
          size={0.4} 
        />
        
        {/* Measured path trail */}
        <MeasuredPath positions={measuredPositions} />
        
        <OrbitControls 
          enablePan={true}
          enableZoom={true}
          enableRotate={true}
          maxDistance={50}
          minDistance={5}
          autoRotate={false}
        />
      </Canvas>
      
      {/* Date/Time display at bottom */}
      <div style={{
        position: 'absolute',
        bottom: '20px',
        left: '50%',
        transform: 'translateX(-50%)',
        background: 'rgba(0,0,0,0.8)',
        color: 'white',
        padding: '10px 20px',
        borderRadius: '8px',
        border: '1px solid rgba(255,255,255,0.2)',
        fontFamily: 'monospace',
        fontSize: '14px'
      }}>
        <div style={{ textAlign: 'center' }}>
          <div style={{ marginBottom: '5px' }}>
            {currentTime ? format(currentTime, 'MMM dd, yyyy').toUpperCase() : 'AUG 11, 2025'}
          </div>
          <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
            <span style={{ color: '#4caf50' }}>REAL RATE</span>
            <div style={{
              width: '8px',
              height: '8px',
              borderRadius: '50%',
              backgroundColor: '#4caf50',
              marginLeft: '5px'
            }}></div>
            <span>{currentTime ? format(currentTime, 'hh:mm:ss a') : '02:20:01 PM'}</span>
          </div>
        </div>
      </div>
    </div>
  );
};

// Control panel component
const ControlPanel = ({ 
  status, 
  wsStatus,
  wsUrl,
  onWsUrlChange,
  onReconnect,
  observerLat,
  observerLon,
  observerAltKm,
  onObserverChange,
  tle1,
  tle2,
  onTleChange,
  onApplyTle,
  predictedPosition, 
  measuredPosition, 
  onClearMeasuredPath 
}) => {
  return (
    <div className="control-panel">
      <div className="info-box">
        <h3>Tracking Info</h3>
        <div className="info-item">
          <label>Status:</label>
          <span>{status}</span>
        </div>
        <div className="info-item">
          <label>Data Link:</label>
          <span>{wsStatus}</span>
        </div>
        <div className="info-item">
          <label>Satellite:</label>
          <span>ISS (25544)</span>
        </div>
        <div className="info-item">
          <label>Current Position (km):</label>
          <span>
            {predictedPosition 
              ? `${(predictedPosition[0] / SCALE_FACTOR).toFixed(0)}, ${(predictedPosition[1] / SCALE_FACTOR).toFixed(0)}, ${(predictedPosition[2] / SCALE_FACTOR).toFixed(0)}`
              : 'N/A'
            }
          </span>
        </div>
        <div className="info-item">
          <label>Visualization:</label>
          <span>🟠 Orange = Predicted (TLE)<br/>🔵 Blue = Measured (LiDAR)<br/>🟢 Green = Orbit path<br/>🔴 Red = Ground-to-target rays</span>
        </div>
      </div>
      
      <div className="info-box">
        <h3>WebSocket</h3>
        <div className="info-item">
          <label>URL:</label>
          <input
            style={{ width: '100%' }}
            value={wsUrl}
            onChange={(e) => onWsUrlChange(e.target.value)}
            placeholder="ws://raspberrypi.local:8765"
          />
        </div>
        <button className="clear-button" onClick={onReconnect}>Reconnect</button>
      </div>

      <div className="info-box">
        <h3>Observer (LiDAR) Location</h3>
        <div className="info-item"><label>Lat (deg):</label><input type="number" step="0.0001" value={observerLat} onChange={(e)=>onObserverChange({ lat: parseFloat(e.target.value) })} /></div>
        <div className="info-item"><label>Lon (deg):</label><input type="number" step="0.0001" value={observerLon} onChange={(e)=>onObserverChange({ lon: parseFloat(e.target.value) })} /></div>
        <div className="info-item"><label>Alt (km):</label><input type="number" step="0.001" value={observerAltKm} onChange={(e)=>onObserverChange({ altKm: parseFloat(e.target.value) })} /></div>
      </div>

      <div className="info-box">
        <h3>TLE</h3>
        <div className="info-item"><label>Line 1</label><input style={{ width: '100%' }} value={tle1} onChange={(e)=>onTleChange(1, e.target.value)} /></div>
        <div className="info-item"><label>Line 2</label><input style={{ width: '100%' }} value={tle2} onChange={(e)=>onTleChange(2, e.target.value)} /></div>
        <button className="clear-button" onClick={onApplyTle}>Apply TLE</button>
      </div>

      <button 
        className="clear-button"
        onClick={onClearMeasuredPath}
      >
        Clear Measured Path
      </button>
    </div>
  );
};

// Main OrbitTracker component
const OrbitTracker = () => {
  const [status, setStatus] = useState('Initializing');
  const [wsStatus, setWsStatus] = useState('disconnected');
  const [predictedPosition, setPredictedPosition] = useState(null);
  const [measuredPosition, setMeasuredPosition] = useState(null);
  const [measuredPositions, setMeasuredPositions] = useState([]);
  const [satelliteObj, setSatelliteObj] = useState(null);
  const [currentTime, setCurrentTime] = useState(new Date());
  const [wsUrl, setWsUrl] = useState('ws://raspberrypi.local:8765');
  const wsRef = useRef(null);
  const [observer, setObserver] = useState({ lat: 37.7749, lon: -122.4194, altKm: 0.02 });
  const [tle, setTle] = useState({
    l1: TLE_LINE1,
    l2: TLE_LINE2,
  });
  // Removed manual points and Kalman filter state
  const [satellites, setSatellites] = useState({});

  // Initialize multiple satellites from TLEs
  useEffect(() => {
    const satObjects = {};
    Object.entries(SATELLITE_TLES).forEach(([id, satData]) => {
      try {
        const rec = satellite.twoline2satrec(satData.tle1, satData.tle2);
        satObjects[id] = {
          ...satData,
          rec,
          position: null,
          color: id === 'ISS' ? '#ff6b35' : '#ffffff'
        };
      } catch (error) {
        console.error(`Error initializing satellite ${id}:`, error);
      }
    });
    setSatellites(satObjects);
    setStatus('Tracking Active');
  }, []);

  // Update satellite positions
  useEffect(() => {
    if (Object.keys(satellites).length === 0) return;

    const updatePositions = () => {
      const now = new Date();
      setCurrentTime(now);
      
      const updatedSatellites = { ...satellites };
      Object.entries(updatedSatellites).forEach(([id, sat]) => {
        if (!sat.rec) return;
        
        try {
          const positionAndVelocity = satellite.propagate(sat.rec, now);
          if (positionAndVelocity.position && !positionAndVelocity.position.error) {
            const cartesianPos = convertEciToCartesian(positionAndVelocity.position);
            updatedSatellites[id].position = cartesianPos;
          }
        } catch (error) {
          console.warn(`Error updating position for ${id}:`, error);
        }
      });
      
      setSatellites(updatedSatellites);
      // Also expose ISS position as predictedPosition for UI display
      if (updatedSatellites.ISS?.position) {
        setPredictedPosition(updatedSatellites.ISS.position);
      }
    };

    const interval = setInterval(updatePositions, 1000); // Update every second
    updatePositions();

    return () => clearInterval(interval);
  }, [satellites]);

  // WebSocket: receive LiDAR readings and convert to scene position
  useEffect(() => {
    if (wsRef.current) {
      wsRef.current.stop();
      wsRef.current = null;
    }
    const client = new SensorWebSocket({
      url: wsUrl,
      onStatusChange: setWsStatus,
      onMessage: (msg) => {
        const { range_m, az_deg, el_deg } = msg;
        try {
          const obsEcef = geodeticToEcef({ latDeg: observer.lat, lonDeg: observer.lon, altKm: observer.altKm || 0 });
          const d = lidarSphericalToEcefDelta({
            rangeMeters: range_m,
            azDegFromNorthCW: az_deg,
            elDeg: el_deg,
            latDeg: observer.lat,
            lonDeg: observer.lon,
          });
          const targetEcef = { x: obsEcef.x + d.dx, y: obsEcef.y + d.dy, z: obsEcef.z + d.dz };
          const scenePos = ecefToSceneArray(targetEcef, SCALE_FACTOR);
          setMeasuredPosition(scenePos);
          setMeasuredPositions(prev => {
            const next = [...prev, scenePos];
            return next.length > 200 ? next.slice(-200) : next;
          });
        } catch (e) {
          // ignore individual conversion errors
        }
      },
    });
    wsRef.current = client;
    client.start();
    return () => client.stop();
  }, [wsUrl, observer.lat, observer.lon, observer.altKm]);

  const onObserverChange = (partial) => {
    setObserver(prev => ({ ...prev, ...partial }));
  };

  const onTleChange = (line, value) => {
    if (line === 1) setTle(prev => ({ ...prev, l1: value }));
    else setTle(prev => ({ ...prev, l2: value }));
  };

  const onApplyTle = () => {
    // Dependency array on tle state triggers re-init
    setTle(prev => ({ ...prev }));
  };

  const reconnectWs = () => {
    if (wsRef.current) {
      wsRef.current.stop();
      wsRef.current.start();
    } else {
      setWsUrl((u) => u);
    }
  };

  const clearMeasuredPath = () => {
    setMeasuredPositions([]);
    setMeasuredPosition(null);
  };

  // Removed manual points and Kalman filter related code

  return (
    <div className="orbit-tracker">
      <div className="header">
        <h1>Real-Time Tracking and Orbit Simulation</h1>
      </div>
      
      <div className="main-content">
        <ControlPanel
          status={status}
          wsStatus={wsStatus}
          wsUrl={wsUrl}
          onWsUrlChange={setWsUrl}
          onReconnect={reconnectWs}
          observerLat={observer.lat}
          observerLon={observer.lon}
          observerAltKm={observer.altKm}
          onObserverChange={onObserverChange}
          tle1={tle.l1}
          tle2={tle.l2}
          onTleChange={onTleChange}
          onApplyTle={onApplyTle}
          predictedPosition={predictedPosition}
          measuredPosition={measuredPosition}
          onClearMeasuredPath={clearMeasuredPath}
        />
        
        <Scene3D
          predictedPosition={predictedPosition}
          measuredPosition={measuredPosition}
          measuredPositions={measuredPositions}
          satelliteRec={satelliteObj}
          currentTime={currentTime}
          satellites={satellites}
        />
      </div>
    </div>
  );
};

export default OrbitTracker;