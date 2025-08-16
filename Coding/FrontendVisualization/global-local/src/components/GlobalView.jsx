import React, { useRef, useEffect, useMemo, Suspense, useState } from 'react';
import { useFrame } from '@react-three/fiber';
import { Html } from '@react-three/drei';
import * as THREE from 'three';
import * as satellite from 'satellite.js';
import { format } from 'date-fns';
import RealisticEarth from './RealisticEarth';
import Starfield from './Starfield';
import OrbitPath from './OrbitPath';
import PredictedOrbit from './PredictedOrbit';
import ErrorBoundary from './ErrorBoundary';

// GlobalView Component - This renders the 3D Earth view with satellites and ground station
// Users can see the entire Earth, track satellites, and click on the ground station to switch to local view

// Ground Station Marker Component - This shows where our LiDAR sensor is located on Earth
// It's positioned at Sofia Tech Park, Bulgaria and rotates with the Earth
const GroundStationMarker = ({ onClick }) => {
  // Sofia Tech Park coordinates in Bulgaria
  const lat = 42.6501;  // Latitude (north-south position)
  const lon = 23.3795;  // Longitude (east-west position)
  const altKm = 0.6;    // Altitude in kilometers above sea level
  
  // Convert GPS coordinates to 3D position on the Earth's surface
  // This ensures the marker stays "pinned" to the correct location as Earth rotates
  const position = useMemo(() => {
    // Earth's radius in kilometers
    const earthRadiusKm = 6371;
    
    // Convert latitude and longitude to radians (math functions expect radians, not degrees)
    const latRad = (lat * Math.PI) / 180;
    const lonRad = (lon * Math.PI) / 180;
    
    // Calculate 3D position on Earth's surface
    // X = distance from center * cos(latitude) * cos(longitude)
    // Y = distance from center * sin(latitude)
    // Z = distance from center * cos(latitude) * sin(longitude)
    const radius = (earthRadiusKm + altKm) / 1000; // Convert to scene units (thousands of km)
    
    return [
      radius * Math.cos(latRad) * Math.cos(lonRad),  // X coordinate
      radius * Math.sin(latRad),                      // Y coordinate (up/down)
      radius * Math.cos(latRad) * Math.sin(lonRad)   // Z coordinate
    ];
  }, [lat, lon, altKm]);

  // This function handles when the user clicks on the ground station marker
  const handleClick = (event) => {
    // Stop the click from affecting the 3D scene (like camera movement)
    event.stopPropagation();
    
    // Call the function passed from the parent to switch to local view
    if (onClick) onClick();
  };

  return (
    <group position={position}>
      {/* Red pyramid marker pointing up from the Earth's surface */}
      <mesh onClick={handleClick}>
        <coneGeometry args={[0.3, 0.8, 4]} />
        <meshBasicMaterial color="#ff4444" />
      </mesh>
      
      {/* Label showing "Sofia Tech Park" above the marker */}
      <Html
        position={[0, 1.2, 0]}
        center
        style={{
          background: 'rgba(0, 0, 0, 0.8)',
          color: 'white',
          padding: '8px 12px',
          borderRadius: '6px',
          fontSize: '12px',
          fontWeight: 'bold',
          whiteSpace: 'nowrap',
          border: '1px solid rgba(255, 68, 68, 0.5)',
          pointerEvents: 'none' // Don't interfere with clicking the marker
        }}
      >
        Sofia Tech Park
      </Html>
    </group>
  );
};

// Satellite Marker Component - This shows individual satellites in 3D space
// Each satellite has a colored sphere and a label with its name
const SatelliteMarker = ({ satellite, position, color, name }) => {
  // If we don't have a position yet, don't render anything
  if (!position) return null;

  return (
    <group position={position}>
      {/* Colored sphere representing the satellite */}
      <mesh>
        <sphereGeometry args={[0.2, 16, 16]} />
        <meshBasicMaterial color={color} />
      </mesh>
      
      {/* Label showing the satellite name */}
      <Html
        position={[0, 0.4, 0]}
        center
        style={{
          background: 'rgba(0, 0, 0, 0.8)',
          color: 'white',
          padding: '6px 10px',
          borderRadius: '4px',
          fontSize: '11px',
          fontWeight: 'bold',
          whiteSpace: 'nowrap',
          border: `1px solid ${color}`,
          pointerEvents: 'none'
        }}
      >
        {name}
      </Html>
    </group>
  );
};



// Main GlobalView Component - This renders the entire 3D Earth scene
const GlobalView = ({
  satellites,                    // Information about satellites we're tracking
  measuredPosition,             // Current LiDAR measurement (hidden in global view)
  positionHistory,              // History of measurements (hidden in global view)
  predictedOrbitParams,         // Calculated orbital parameters for Optaris
  status,                       // Current system status
  onGroundStationClick          // Function to call when ground station is clicked
}) => {
  // Reference to the scene group for managing the 3D objects
  const sceneRef = useRef();
  
  // Current time for satellite position calculations
  const [currentTime, setCurrentTime] = useState(new Date());

  // Update the current time every second
  // This ensures satellite positions are calculated for the current moment
  useEffect(() => {
    const timer = setInterval(() => {
      setCurrentTime(new Date());
    }, 1000);

    return () => clearInterval(timer);
  }, []);

  // This function runs every frame to update satellite positions
  // It recalculates where each satellite should be based on the current time
  useFrame(() => {
    if (!sceneRef.current || !satellites) return;

    // Go through each satellite and update its position
    Object.entries(satellites).forEach(([id, satData]) => {
      if (!satData.rec) return;

      try {
        // Calculate the satellite's position at the current time
        // This uses the TLE data to predict where the satellite is right now
        const positionAndVelocity = satellite.propagate(satData.rec, currentTime);
        
        if (positionAndVelocity.position) {
          // Convert the position from kilometers to scene units
          const pos = positionAndVelocity.position;
          const scenePos = [
            pos.x / 1000,  // Convert km to thousands of km
            pos.y / 1000,
            pos.z / 1000
          ];
          
          // Update the satellite's position in our data
          satData.position = scenePos;
        }
      } catch (error) {
        // If there's an error calculating the position, log it for debugging
        console.error(`Error calculating position for ${id}:`, error);
      }
    });
  });

  // Calculate the current time as a Julian Date (astronomical time format)
  // This is what the satellite library uses for calculations
  const julianDate = useMemo(() => {
    return currentTime.getTime() / 86400000 + 2440587.5;
  }, [currentTime]);

  return (
    <group ref={sceneRef}>
      {/* Lighting for the 3D scene */}
      <ambientLight intensity={0.3} />
      <directionalLight position={[10, 10, 5]} intensity={1} />
      <pointLight position={[-10, -10, -5]} intensity={0.5} />
      
      {/* Starfield background - wrapped in error boundary */}
      <ErrorBoundary>
        <Starfield />
      </ErrorBoundary>
      
      {/* 3D Earth model - wrapped in error boundary */}
      <ErrorBoundary>
        <RealisticEarth />
      </ErrorBoundary>
      
  {/* Ground station marker removed as per request */}
      
      {/* Render each satellite except ISS and Optaris (strict match) */}
      {Object.entries(satellites)
        .filter(([id, satData]) => {
          const name = (satData.name || '').trim().toLowerCase();
          return name !== 'international space station' && name !== 'iss' && name !== 'optaris';
        })
        .map(([id, satData]) => (
          <SatelliteMarker
            key={id}
            satellite={satData}
            position={satData.position}
            color={satData.color}
            name={satData.name}
          />
        ))}
      
      {/* Render orbit paths for satellites except ISS, Optaris, and equator (strict match) */}
      {Object.entries(satellites)
        .filter(([id, satData]) => {
          const name = (satData.name || '').trim().toLowerCase();
          return name !== 'international space station' && name !== 'iss' && name !== 'optaris' && !name.includes('equator');
        })
        .map(([id, satData]) => {
          if (!satData.rec) return null;
          return (
            <ErrorBoundary key={`orbit-${id}`}>
              <OrbitPath
                orbitParams={{
                  elements: {
                    semiMajorAxis: 7000, // Approximate orbit size in km
                    eccentricity: 0.001,  // Nearly circular orbit
                    inclinationRad: 0.9,  // Orbit tilt in radians
                    // Add other orbital elements as needed
                  }
                }}
                color={satData.color}
                segments={150}
              />
            </ErrorBoundary>
          );
        })}
      
      {/* Predicted orbit from LiDAR measurements */}
      <ErrorBoundary>
        <PredictedOrbit 
          predictedOrbitParams={predictedOrbitParams}
          status={status}
        />
      </ErrorBoundary>
      
      {/* Current time display in 3D space */}
      <Html
        position={[0, 15, 0]}
        center
        style={{
          background: 'rgba(0, 0, 0, 0.7)',
          color: 'white',
          padding: '10px 15px',
          borderRadius: '8px',
          fontSize: '14px',
          fontWeight: 'bold',
          border: '1px solid rgba(255, 255, 255, 0.3)',
          pointerEvents: 'none'
        }}
      >
        {format(currentTime, 'HH:mm:ss')} UTC
      </Html>
    </group>
  );
};

// Export the GlobalView component so it can be used by the main App component
export default GlobalView;
