// src/useDroneStore.js
// This file contains the global state management for the drone tracker application
// Uses Zustand library for lightweight, hook-based state management

// Import the create function from Zustand to build our store
import { create } from 'zustand';

// Create the main drone store with state and actions
// Zustand provides 'set' for updating state and 'get' for reading current state
const useDroneStore = create((set, get) => ({
  // === INITIAL STATE DEFINITION ===
  
  // Current connection and system status (e.g., 'CONNECTING...', 'RUNNING', 'ERROR')
  status: 'CONNECTING...',
  
  // Progress percentage for operations that show progress (0-100)
  progress: 0,
  
  // Warning message from the system (null if no warning)
  warning: null,

  // Current coordinates from tracking system
  coordinates: null,

  // Predicted trajectory data
  prediction: null,
  
  // Error message from the system (null if no error)
  error: null,
  
  // Main data payload containing all drone-related information
  payload: {
    // Current measured position as [x, y, z] coordinates (null values if not available)
    measured_pos: [null, null, null],
    
    // Where the scanner is currently pointing as [x, y, z]
    scanner_pos: [null, null, null],
    
    // Live telemetry data from the drone (distance, signal strength, angles, etc.)
    live_telemetry: {},
  },

  // Action to update coordinates
  updateCoordinates: (coordinates) => set((state) => ({ 
    coordinates,
    payload: {
      ...state.payload,
      measured_pos: [coordinates.x, coordinates.y, coordinates.z],
      scanner_pos: [coordinates.azimuth, coordinates.elevation, coordinates.distance]
    }
  })),

  // Action to update prediction
  updatePrediction: (prediction) => set((state) => ({ 
    ...state,
    prediction 
  })),
    
    // Array of previous positions with timestamps for flight path visualization
    position_history: [],
    
    // Predicted orbital parameters for showing future drone path (computed on frontend)
    predicted_orbit_params: null,
    
    // Initial orbit parameters derived from TLE at tracking start (from backend)
    initial_orbit_params: null,

    // Two-Line Element set provided at tracking start (from backend)
    initial_tle: null,
  },
  
  // === ACTIONS ===
  
  /**
   * Safely merges new data from backend into the store, with defensive checks.
   */
  updateData: (data) => {
    // The 'get' function gives us access to the current state
    const currentState = get();

    // Safely update position history only if it exists in the new data
    let updatedHistory = currentState.payload.position_history;
    if (data.payload?.position_history) {
      // Get current timestamp for when this position was recorded
      const now = Date.now();
      // This creates a new array with fresh timestamps for each position
      // Each position gets the same timestamp since they're all from the same update
      updatedHistory = data.payload.position_history.map(pos => ({ pos, time: now }));
    }

    // Merge the incoming data into state
    set({
      // Use new value if it exists, otherwise keep the old one
      status: data.status ?? currentState.status,
      progress: data.progress ?? currentState.progress,
      // Preserve existing warning/error if not explicitly provided; allow null to clear
      warning: ("warning" in data) ? data.warning : currentState.warning,
      error: ("error" in data) ? data.error : currentState.error,
      prediction_warning: null, // Clear warning on new data

      // Deeply merge the payload to avoid losing data
      payload: {
        ...currentState.payload, // Start with the old payload (spread operator)
        ...(data.payload || {}), // Spread new payload data if it exists, empty object if not
        position_history: updatedHistory, // Overwrite with our timestamped history
      },
    });

    // After merging, compute/update the predicted orbit if enough data exists
    // This keeps prediction live without blocking the UI
    get().computePredictedOrbitFromHistory();
  },

  /**
   * Clears state for a front-end reset without needing backend data immediately.
   * Sets status to 'Idle' and removes history, warnings, errors, and orbits.
   */
  clearForReset: () => {
    // Use 'set' to update multiple parts of the state at once
    set((state) => ({
      status: 'Idle', // Set user-visible idle status after reset
      progress: 0,    // Reset progress bar to zero
      warning: null,  // Clear any warning messages
      error: null,    // Clear any error messages
      payload: {
        ...state.payload,                   // Keep other payload fields intact
        measured_pos: [null, null, null],   // Clear current measured position
        scanner_pos: [null, null, null],    // Clear current scanner target
        live_telemetry: {},                 // Clear telemetry values
        position_history: [],               // Empty position history
        predicted_orbit_params: null,       // Remove predicted orbit
        initial_orbit_params: null,         // Remove initial orbit info
        initial_tle: null,                  // Remove TLE info
      },
    }));
  },

  /**
   * Computes a placeholder predicted orbit from recent position_history points.
   * This is a stand-in for a future Kalman filter implementation.
   */
  computePredictedOrbitFromHistory: () => {
    // Read the latest state snapshot
    const { payload } = get();
    // Read the position history array of objects { pos: [x,y,z], time }
    const history = payload.position_history || [];
    let prediction_warning = null;
    // Require at least three points to compute any meaningful orbit estimate
    if (!Array.isArray(history) || history.length < 3) {
      prediction_warning = 'Not enough data for orbit prediction.';
      set((state) => ({
        prediction_warning,
        payload: {
          ...state.payload,
          predicted_orbit_params: null,
        },
      }));
      return;
    }

    // Create a shallow copy and take the most recent up to 100 data points
    const recent = history.slice(-100);
    // Extract arrays of x, y, z values for computations
    const xs = recent.map((h) => h.pos[0]);
    const ys = recent.map((h) => h.pos[1]);
    const zs = recent.map((h) => h.pos[2]);

    // Compute simple statistics: min, max, and averages for each axis
    const minX = Math.min(...xs); const maxX = Math.max(...xs);
    const minZ = Math.min(...zs); const maxZ = Math.max(...zs);
    const avgY = ys.reduce((a, b) => a + b, 0) / ys.length;

    // Estimate ellipse radii in X and Z from the spread (half-range)
    let radiusX = Math.max(0.1, (maxX - minX) / 2);
    let radiusZ = Math.max(0.1, (maxZ - minZ) / 2);

    // Identify semi-major a and semi-minor b axes (largest is 'a')
    const a_major = Math.max(radiusX, radiusZ);
    const b_minor = Math.min(radiusX, radiusZ);

    // Compute a simple eccentricity estimate for an ellipse: e = sqrt(1 - (b^2/a^2))
    const eccentricity = Math.sqrt(Math.max(0, 1 - (b_minor * b_minor) / (a_major * a_major)));

    // Estimate inclination by fitting a plane normal from first three points
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
    // Inclination as angle between normal and global Y axis (in radians)
    const dotWithY = normal[1] / normalMag;
    const inclinationRad = Math.acos(Math.min(1, Math.max(-1, Math.abs(dotWithY))));

    // Compute simple angles for RAAN and argument of periapsis (placeholders)
    const last = recent[recent.length - 1].pos;
    const trueAnomalyRad = Math.atan2(last[2], last[0]);
    const raanRad = 0;              // Placeholder (unknown without full orbital model)
    const argPeriapsisRad = 0;      // Placeholder (unknown without pericenter solution)

    // Add a warning if the data is noisy (e.g., large std deviation)
    const std = (arr) => {
      const mean = arr.reduce((a, b) => a + b, 0) / arr.length;
      return Math.sqrt(arr.reduce((a, b) => a + (b - mean) ** 2, 0) / arr.length);
    };
    if (std(xs) > 10 || std(ys) > 10 || std(zs) > 10) {
      prediction_warning = 'Predicted orbit may be inaccurate due to noisy data.';
    }

    // Build a predicted orbit params object for 3D display and HUD
    const predicted = {
      type: 'ellipse',                       // Tell the scene to render an ellipse line
      a: Number(a_major.toFixed(2)),         // Semi-major axis for visualization (in scene units)
      b: Number(b_minor.toFixed(2)),         // Semi-minor axis for visualization (in scene units)
      altitude: Number(avgY.toFixed(2)),     // Average altitude as Y plane for the ellipse
      elements: {                            // Classical orbital elements (approximate)
        semiMajorAxis: Number(a_major.toFixed(2)),
        eccentricity: Number(eccentricity.toFixed(3)),
        inclinationDeg: Number((inclinationRad * 180 / Math.PI).toFixed(2)),
        raanDeg: Number((raanRad * 180 / Math.PI).toFixed(2)),
        argPeriapsisDeg: Number((argPeriapsisRad * 180 / Math.PI).toFixed(2)),
        trueAnomalyDeg: Number((trueAnomalyRad * 180 / Math.PI).toFixed(2)),
      },
    };

    // Save the new prediction into the store for both HUD and Scene to use
    set((state) => ({
      prediction_warning,
      payload: {
        ...state.payload,
        predicted_orbit_params: predicted,
      },
    }));
  },
}));

// Export the store hook for use in React components
export default useDroneStore;