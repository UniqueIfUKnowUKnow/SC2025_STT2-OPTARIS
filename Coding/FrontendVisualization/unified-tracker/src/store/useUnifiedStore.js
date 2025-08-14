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
    distance: null
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
      if (payload.position_history) {
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
        distance: null
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
  
  computePredictedOrbit: () => {
    const { positionHistory } = get();
    
    if (!Array.isArray(positionHistory) || positionHistory.length < 3) {
      set({ predictedOrbitParams: null });
      return;
    }
    
    // Take the most recent points
    const recent = positionHistory.slice(-100);
    const xs = recent.map(h => h.pos[0]);
    const ys = recent.map(h => h.pos[1]);
    const zs = recent.map(h => h.pos[2]);
    
    // Compute simple statistics
    const minX = Math.min(...xs);
    const maxX = Math.max(...xs);
    const minZ = Math.min(...zs);
    const maxZ = Math.max(...zs);
    const avgY = ys.reduce((a, b) => a + b, 0) / ys.length;
    
    // Estimate ellipse parameters
    let radiusX = Math.max(0.1, (maxX - minX) / 2);
    let radiusZ = Math.max(0.1, (maxZ - minZ) / 2);
    
    const a_major = Math.max(radiusX, radiusZ);
    const b_minor = Math.min(radiusX, radiusZ);
    const eccentricity = Math.sqrt(Math.max(0, 1 - (b_minor * b_minor) / (a_major * a_major)));
    
    // Estimate inclination
    const p0 = recent[Math.max(0, recent.length - 3)].pos;
    const p1 = recent[Math.max(0, recent.length - 2)].pos;
    const p2 = recent[Math.max(0, recent.length - 1)].pos;
    const v1 = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
    const v2 = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
    const normal = [
      v1[1] * v2[2] - v1[2] * v2[1],
      v1[2] * v2[0] - v1[0] * v2[2],
      v1[0] * v2[1] - v1[1] * v2[0],
    ];
    const normalMag = Math.sqrt(normal[0]**2 + normal[1]**2 + normal[2]**2) || 1;
    const dotWithY = normal[1] / normalMag;
    const inclinationRad = Math.acos(Math.min(1, Math.max(-1, Math.abs(dotWithY))));
    
    const predicted = {
      type: 'ellipse',
      a: Number(a_major.toFixed(2)),
      b: Number(b_minor.toFixed(2)),
      altitude: Number(avgY.toFixed(2)),
      elements: {
        semiMajorAxis: Number(a_major.toFixed(2)),
        eccentricity: Number(eccentricity.toFixed(3)),
        inclinationDeg: Number((inclinationRad * 180 / Math.PI).toFixed(2)),
        raanDeg: 0, // Placeholder
        argPeriapsisDeg: 0, // Placeholder
        trueAnomalyDeg: 0, // Placeholder
      },
    };
    
    set({ predictedOrbitParams: predicted });
  },
}));

export default useUnifiedStore;
