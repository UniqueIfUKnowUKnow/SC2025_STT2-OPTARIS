// src/useDroneStore.js
import { create } from 'zustand';

const useDroneStore = create((set, get) => ({
  // The initial state of the application
  status: 'CONNECTING...',
  progress: 0,
  warning: null,
  error: null,
  payload: {
    measured_pos: [null, null, null],
    live_telemetry: {},
    position_history: [],
    predicted_orbit_params: null,
  },
  
  /**
   * This is the corrected, defensive update function.
   * It safely merges new data with the existing state,
   * preventing crashes from incomplete data packets.
   */
  updateData: (data) => {
    // The 'get' function gives us access to the current state
    const currentState = get();

    // Safely update position history only if it exists in the new data
    let updatedHistory = currentState.payload.position_history;
    if (data.payload?.position_history) {
      const now = Date.now();
      // This creates a new array with fresh timestamps
      updatedHistory = data.payload.position_history.map(pos => ({ pos, time: now }));
    }

    set({
      // Use new value if it exists, otherwise keep the old one
      status: data.status ?? currentState.status,
      progress: data.progress ?? currentState.progress,
      warning: data.warning, // Allow setting warning to null
      error: data.error,     // Allow setting error to null

      // Deeply merge the payload to avoid losing data
      payload: {
        ...currentState.payload, // Start with the old payload
        ...(data.payload || {}), // Spread new payload data if it exists
        position_history: updatedHistory, // Overwrite with our timestamped history
      },
    });
  },
}));

export default useDroneStore;