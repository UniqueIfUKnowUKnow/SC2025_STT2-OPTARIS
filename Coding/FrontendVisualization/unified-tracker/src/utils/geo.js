// Simple geospatial utilities using a spherical Earth model

export const EARTH_RADIUS_KM = 6371;

export function degToRad(deg) {
  return (deg * Math.PI) / 180;
}

export function radToDeg(rad) {
  return (rad * 180) / Math.PI;
}

// ECEF coordinates (km) for a geodetic position on a spherical Earth
export function geodeticToEcef({ latDeg, lonDeg, altKm = 0 }) {
  const lat = degToRad(latDeg);
  const lon = degToRad(lonDeg);
  const r = EARTH_RADIUS_KM + altKm;
  const x = r * Math.cos(lat) * Math.cos(lon);
  const y = r * Math.cos(lat) * Math.sin(lon);
  const z = r * Math.sin(lat);
  return { x, y, z };
}

// Local ENU basis unit vectors in ECEF frame at given geodetic location
export function enuBasisAt({ latDeg, lonDeg }) {
  const lat = degToRad(latDeg);
  const lon = degToRad(lonDeg);
  // East
  const E = {
    x: -Math.sin(lon),
    y: Math.cos(lon),
    z: 0,
  };
  // North
  const N = {
    x: -Math.sin(lat) * Math.cos(lon),
    y: -Math.sin(lat) * Math.sin(lon),
    z: Math.cos(lat),
  };
  // Up
  const U = {
    x: Math.cos(lat) * Math.cos(lon),
    y: Math.cos(lat) * Math.sin(lon),
    z: Math.sin(lat),
  };
  return { E, N, U };
}

// Convert LiDAR spherical reading (azimuth from North, clockwise; elevation above horizon) to ECEF delta vector (km)
export function lidarSphericalToEcefDelta({ rangeMeters, azDegFromNorthCW, elDeg, latDeg, lonDeg }) {
  const rangeKm = Math.max(0, rangeMeters) / 1000;
  const az = degToRad(azDegFromNorthCW);
  const el = degToRad(elDeg);

  const { E, N, U } = enuBasisAt({ latDeg, lonDeg });

  // ENU components for az measured clockwise from North
  const east = Math.cos(el) * Math.sin(az);
  const north = Math.cos(el) * Math.cos(az);
  const up = Math.sin(el);

  // Combine into ECEF
  const dx = rangeKm * (east * E.x + north * N.x + up * U.x);
  const dy = rangeKm * (east * E.y + north * N.y + up * U.y);
  const dz = rangeKm * (east * E.z + north * N.z + up * U.z);
  return { dx, dy, dz };
}

// Map ECEF to scene coordinates used in the app
export function ecefToSceneArray({ x, y, z }, scaleFactor) {
  return [x * scaleFactor, z * scaleFactor, -y * scaleFactor];
}
