const WebSocket = require('ws');

// Create WebSocket server on port 8080
const wss = new WebSocket.Server({ port: 8080 });

// Function to generate random points within the 12-meter sphere
function generateRandomPoints(count) {
  const points = [];
  for (let i = 0; i < count; i++) {
    // Generate random spherical coordinates
    const radius = Math.random() * 12; // Random radius up to 12 meters
    const theta = Math.random() * Math.PI * 2; // Random angle around Y axis
    const phi = Math.acos((Math.random() * 2) - 1); // Random angle from Y axis

    // Convert spherical to Cartesian coordinates
    const x = radius * Math.sin(phi) * Math.cos(theta);
    const y = radius * Math.sin(phi) * Math.sin(theta);
    const z = radius * Math.cos(phi);

    points.push({ x, y, z });
  }
  return points;
}

// Handle client connections
wss.on('connection', (ws) => {
  console.log('Client connected');

  // Send new points every second
  const interval = setInterval(() => {
    const points = generateRandomPoints(10); // Generate 10 random points
    ws.send(JSON.stringify(points));
  }, 1000);

  // Handle client disconnection
  ws.on('close', () => {
    console.log('Client disconnected');
    clearInterval(interval);
  });

  // Handle errors
  ws.on('error', (error) => {
    console.error('WebSocket error:', error);
    clearInterval(interval);
  });
});

console.log('WebSocket server running on ws://localhost:8080');
