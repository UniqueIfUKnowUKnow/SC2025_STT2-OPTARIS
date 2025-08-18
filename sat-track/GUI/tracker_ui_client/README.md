# Unified Tracker (Vite + React + R3F)

3D viewer that shows a ground station on Earth, satellite orbits from TLEs, and a target/drone trajectory from WebSocket data.

## What you see
- **Ground station**: fixed at Sofia Tech Park, BG.
- **Initial TLE orbit**: blue line (propagated from initial TLE).
- **New TLE orbit**: pink line (propagated from updated TLE).
- **Points trajectory**: yellow dashed line (array of ECEF points with timestamps).
- **Predicted orbit (local)**: green line estimated from LiDAR history.

## Quick start
```bash
npm ci
npm run dev   # http://localhost:3000

# Production
npm run build
npm run preview
```

## Modes
- **Simulation**: default; no network required.
- **Real**: select in the HUD, set WebSocket URL (e.g. `ws://<pi-ip>:8765`). In Real mode, Start/Stop/Reset are controlled by the Raspberry Pi.

## WebSocket payloads (concise)
Send JSON messages to the frontend. Supported fields are optional; send only what you have.

### Initial TLE
Any of the following shapes are accepted:
```json
{ "initial_tle1": "...", "initial_tle2": "..." }
{ "init_tle1": "...", "init_tle2": "..." }
{ "initial": { "tle1": "...", "tle2": "..." } }
```

### New/updated TLE
```json
{ "new_tle1": "...", "new_tle2": "..." }
{ "updated_tle1": "...", "updated_tle2": "..." }
{ "new": { "tle1": "...", "tle2": "..." } }
```

### Trajectory points (ECEF)
Units: kilometers. The app converts ECEF to scene coordinates.
```json
{
  "trajectory_points": [
    { "x": 1234.5, "y": -987.6, "z": 5432.1, "timestamp": 1712345678901 },
    { "ecef": { "x": 1235.2, "y": -988.0, "z": 5433.0 }, "t": 1712345679901 }
  ]
}
```

### Status and telemetry (typical unified format)
```json
{
  "status": "TRACKING_TARGET",
  "progress": 42,
  "scanner_pos": [x, y, z],
  "measured_pos": [x, y, z],
  "live_telemetry": { "pan_angle": 12.3, "tilt_angle": 4.5, "distance": 2.1 },
  "position_history": [ { "position": [x, y, z], "timestamp": 1712345 } ],
  "predicted_orbit_params": { /* optional */ }
}
```

## Controls
- Orbit/pan/zoom in 3D with mouse.
- Click the ground station in Global view to jump to Local view.
- HUD: switch views, set mode/URL, view status/telemetry.

## Coordinates & scale
- Incoming trajectory points: ECEF (km). Converted to scene with scale 0.001 (≈ thousands of km in Global view).
- Local view uses meters around the LiDAR origin.

## Files of interest
- `src/store/useUnifiedStore.js`: parses TLEs, stores trajectory, handles WebSocket updates.
- `src/components/TLEOrbitPath.jsx`: propagates and draws TLE orbits.
- `src/components/TrajectoryLine.jsx`: draws received trajectory points.
- `src/components/GlobalView.jsx`: renders Earth, ground station, TLE orbits, trajectory.

## Troubleshooting
- No orbit lines: check that both `tle1` and `tle2` arrive and parse.
- No trajectory: verify `trajectory_points` array and ECEF units (km).
- Connection issues: confirm Pi IP/port, firewall, and that WebSocket server is running.

—
Short, precise README for the Vite React app showing TLE orbits and trajectories.
