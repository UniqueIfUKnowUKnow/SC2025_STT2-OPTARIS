import { create } from 'zustand';
import { geodeticToEcef, lidarSphericalToEcefDelta, ecefToSceneArray, EARTH_RADIUS_KM } from '../utils/geo';

// Sofia Tech Park, Bulgaria coordinates
const SOFIA_TECH_PARK = {
  lat: 42.6501,
  lon: 23.3795,
  altKm: 0.6
};

const SCALE_FACTOR = 0.001; // Scale factor for global view

const useUnifiedStore = create((set, get) => ({
  // === INITIAL STATE ===
  
  // System status
  status: 'STANDBY',
  progress: 0,
  warning: null,
  error: null,
  
  // Position data
  measuredPosition: null, // Current measured position [x, y, z]
  scannerPosition: [null, null, null], // Where scanner is pointing [x, y, z]
  
  // Telemetry
  liveTelemetry: {
    pan_angle: null,
    tilt_angle: null,
    signal_strength: null,
    distance: null,
    statusMessage: null // NEW: Status message from WebSocket payload
  },
  
  // History and tracking
  positionHistory: [], // Array of { pos: [x,y,z], time: timestamp }
  predictedOrbitParams: null, // Computed orbital parameters
  
  // Satellites
  satellites: {},
  
  // Ground station location
  groundStation: SOFIA_TECH_PARK,
  
  // === ACTIONS ===
  
  updateData: (data) => {
    const currentState = get();
    
    // NEW: Handle the new WebSocket message format from Raspberry Pi
    if (data.state) {
      // New format: { state: "CALIBRATING|SCANNING|DETECTED|FINISHED", payload: {...} }
      const { state, payload } = data;
      
      // Map the new state values to existing status values
      let mappedStatus = state;
      let mappedProgress = payload.progress || 0;
      let mappedWarning = null;
      let mappedError = null;
      
      // Process state-specific data
      switch (state) {
        case 'CALIBRATING':
          mappedStatus = 'BACKGROUND_SCAN_IN_PROGRESS';
          // Handle calibration progress
          if (payload.status === 'complete' && payload.average_distance) {
            mappedWarning = `Calibration complete! Average background distance: ${payload.average_distance} cm`;
          }
          break;
          
        case 'SCANNING':
          mappedStatus = 'SEARCHING_FOR_TARGET';
          // Handle real-time scanning data
          if (payload.angle !== undefined) {
            // Convert angle to scanner position for visualization
            const scannerPos = get().calculateScannerPosition(payload.angle);
            set(state => ({
              scannerPosition: scannerPos,
              liveTelemetry: {
                ...state.liveTelemetry,
                pan_angle: payload.angle,
                distance: payload.distance,
                statusMessage: payload.message
              }
            }));
          }
          break;
          
        case 'DETECTED':
          mappedStatus = 'TRACKING_TARGET';
          // Handle final detection data
          if (payload.distance && payload.stepper_angle) {
            // Convert detection data to position
            const targetPos = get().calculateTargetPosition(payload);
            set(state => ({
              measuredPosition: targetPos,
              statusMessage: payload.message
            }));
            
            // Add to position history
            const newHistoryEntry = {
              pos: targetPos,
              time: Date.now()
            };
            set(state => ({
              positionHistory: [...state.positionHistory.slice(-49), newHistoryEntry]
            }));
          }
          break;
          
        case 'FINISHED':
          mappedStatus = 'STANDBY';
          mappedWarning = payload.message || 'Process completed';
          break;
          
        default:
          // Unknown state, keep current status
          mappedStatus = currentState.status;
      }
      
      // Update the store with the new data
      set({
        status: mappedStatus,
        progress: mappedProgress,
        warning: mappedWarning,
        error: mappedError,
        liveTelemetry: {
          ...currentState.liveTelemetry,
          statusMessage: payload.message || currentState.liveTelemetry.statusMessage
        }
      });
      
      console.log('Store updated with new WebSocket format:', {
        state,
        mappedStatus,
        progress: mappedProgress,
        message: payload.message
      });
      
      return;
    }
    
    // Handle the unified format from the backend
    // The backend sends data directly without a payload wrapper
    if (data.status !== undefined) {
      // Extract data directly from the root level
      const {
        status,
        progress,
        warning,
        error,
        scanner_pos,
        measured_pos,
        live_telemetry,
        position_history,
        predicted_orbit_params
      } = data;
      
      // Update position history with timestamps
      let updatedHistory = currentState.positionHistory;
      if (position_history && Array.isArray(position_history)) {
        updatedHistory = position_history.map(entry => ({
          pos: entry.position || entry.pos,
          time: entry.timestamp || Date.now()
        }));
      }
      
      // Convert measured position from LiDAR coordinates to scene coordinates
      let measuredPos = null;
      if (measured_pos && measured_pos[0] !== null) {
        try {
          const obsEcef = geodeticToEcef({ 
            latDeg: SOFIA_TECH_PARK.lat, 
            lonDeg: SOFIA_TECH_PARK.lon, 
            altKm: SOFIA_TECH_PARK.altKm 
          });
          const scenePos = ecefToSceneArray(obsEcef, SCALE_FACTOR);
          measuredPos = scenePos;
        } catch (error) {
          console.warn('Error converting measured position:', error);
        }
      }
      
      set({
        status: status ?? currentState.status,
        progress: progress ?? currentState.progress,
        warning: "warning" in data ? warning : currentState.warning,
        error: "error" in data ? error : currentState.error,
        
        measuredPosition: measuredPos,
        scannerPosition: scanner_pos || currentState.scannerPosition,
        liveTelemetry: live_telemetry || currentState.liveTelemetry,
        positionHistory: updatedHistory,
        predictedOrbitParams: predicted_orbit_params || currentState.predictedOrbitParams,
      });
      
      console.log('Store updated with data:', {
        status,
        progress,
        scanner_pos,
        measured_pos,
        live_telemetry
      });
      
    } else if (data.payload) {
      // Legacy format with payload wrapper (keep for backward compatibility)
      const payload = data.payload;
      
      // Update position history with timestamps
      let updatedHistory = currentState.positionHistory;
      if (payload.position_history && Array.isArray(payload.position_history)) {
        const now = Date.now();
        updatedHistory = payload.position_history.map(pos => ({ pos, time: now }));
      }
      
      // Convert measured position from LiDAR coordinates to scene coordinates
      let measuredPos = null;
      if (payload.measured_pos && payload.measured_pos[0] !== null) {
        try {
          const obsEcef = geodeticToEcef({ 
            latDeg: SOFIA_TECH_PARK.lat, 
            lonDeg: SOFIA_TECH_PARK.lon, 
            altKm: SOFIA_TECH_PARK.altKm 
          });
          const scenePos = ecefToSceneArray(obsEcef, SCALE_FACTOR);
          measuredPos = scenePos;
        } catch (error) {
          console.warn('Error converting measured position:', error);
        }
      }
      
      set({
        status: data.status ?? currentState.status,
        progress: data.progress ?? currentState.progress,
        warning: "warning" in data ? data.warning : currentState.warning,
        error: "error" in data ? data.error : currentState.error,
        
        measuredPosition: measuredPos,
        scannerPosition: payload.scanner_pos || currentState.scannerPosition,
        liveTelemetry: payload.live_telemetry || currentState.liveTelemetry,
        positionHistory: updatedHistory,
        predictedOrbitParams: payload.predicted_orbit_params || currentState.predictedOrbitParams,
      });
    } else if (data.range_m !== undefined) {
      // Legacy LiDAR format
      const { range_m, az_deg, el_deg } = data;
      try {
        const obsEcef = geodeticToEcef({ 
          latDeg: SOFIA_TECH_PARK.lat, 
          lonDeg: SOFIA_TECH_PARK.lon, 
          altKm: SOFIA_TECH_PARK.altKm 
        });
        const d = lidarSphericalToEcefDelta({
          rangeMeters: range_m,
          azDegFromNorthCW: az_deg,
          elDeg: el_deg,
          latDeg: SOFIA_TECH_PARK.lat,
          lonDeg: SOFIA_TECH_PARK.lon,
        });
        const targetEcef = { 
          x: obsEcef.x + d.dx, 
          y: obsEcef.y + d.dy, 
          z: obsEcef.z + d.dz 
        };
        const scenePos = ecefToSceneArray(targetEcef, SCALE_FACTOR);
        
        set(state => ({
          measuredPosition: scenePos,
          positionHistory: [...state.positionHistory.slice(-49), { pos: scenePos, time: Date.now() }]
        }));
      } catch (error) {
        console.warn('Error processing LiDAR data:', error);
      }
    }
    
    // Compute predicted orbit if enough data
    get().computePredictedOrbit();
  },
  
  // NEW: Helper function to calculate scanner position from angle
  calculateScannerPosition: (angle) => {
    // Convert angle to 3D position for visualization
    // This is a simplified calculation - adjust based on your scanner geometry
    const radius = 5; // Scanner arm length in meters
    const x = radius * Math.cos((angle * Math.PI) / 180);
    const y = 0; // Height remains constant
    const z = radius * Math.sin((angle * Math.PI) / 180);
    return [x, y, z];
  },
  
  // NEW: Helper function to calculate target position from detection data
  calculateTargetPosition: (payload) => {
    // Convert detection data to 3D position
    // This is a simplified calculation - adjust based on your coordinate system
    const { distance, stepper_angle, servo_angle } = payload;
    
    // Convert distance from cm to meters
    const distanceM = distance / 100;
    
    // Calculate position based on angles and distance
    const x = distanceM * Math.cos((stepper_angle * Math.PI) / 180);
    const y = distanceM * Math.sin((servo_angle * Math.PI) / 180);
    const z = distanceM * Math.sin((stepper_angle * Math.PI) / 180);
    
    return [x, y, z];
  },
  
  clearForReset: () => {
    set({
      status: 'STANDBY',
      progress: 0,
      warning: null,
      error: null,
      measuredPosition: null,
      scannerPosition: [null, null, null],
      liveTelemetry: {
        pan_angle: null,
        tilt_angle: null,
        signal_strength: null,
        distance: null,
        statusMessage: null
      },
      positionHistory: [],
      predictedOrbitParams: null,
    });
  },
  
  setStatus: (status) => set({ status }),
  setProgress: (progress) => set({ progress }),
  setWarning: (warning) => set({ warning }),
  setError: (error) => set({ error }),
  
  setSatellites: (satellites) => set({ satellites }),
  
  // === ORBITAL CALCULATIONS ===
  
  computePredictedOrbit: () => {
    const state = get();
    if (!state.measuredPosition || state.positionHistory.length < 3) return;
    
    try {
      // Calculate orbital parameters from position history
      // This is a simplified calculation - implement proper orbital mechanics as needed
      const positions = state.positionHistory.map(h => h.pos);
      const center = positions.reduce((acc, pos) => 
        [acc[0] + pos[0], acc[1] + pos[1], acc[2] + pos[2]], [0, 0, 0]
      ).map(coord => coord / positions.length);
      
      // Calculate average distance from center
      const avgDistance = positions.reduce((acc, pos) => {
        const dx = pos[0] - center[0];
        const dy = pos[1] - center[1];
        const dz = pos[2] - center[2];
        return acc + Math.sqrt(dx*dx + dy*dy + dz*dz);
      }, 0) / positions.length;
      
      const orbitParams = {
        elements: {
          semiMajorAxis: avgDistance * 1000, // Convert to km
          eccentricity: 0.1, // Simplified
          inclination: 45, // Simplified
          argumentOfPerigee: 0, // Simplified
          rightAscensionOfAscendingNode: 0, // Simplified
          meanAnomaly: 0 // Simplified
        },
        center: center,
        radius: avgDistance
      };
      
      set({ predictedOrbitParams: orbitParams });
    } catch (error) {
      console.warn('Error computing predicted orbit:', error);
    }
  },
}));

export default useUnifiedStore;
