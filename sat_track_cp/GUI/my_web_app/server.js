// Import the WebSocketServer and WebSocket classes from the 'ws' library.
import { WebSocketServer, WebSocket } from 'ws';
import http from 'http';

// Try to start the server on the first available port, beginning at env or 8080
const START_PORT = Number(process.env.WS_PORT || 8080);
const MAX_PORT = START_PORT + 10;

function startServer(port) {
    const server = http.createServer();

    server.on('error', (err) => {
        if (err && err.code === 'EADDRINUSE') {
            const next = port + 1;
            if (next <= MAX_PORT) {
                console.warn(`Port ${port} in use, trying ${next}...`);
                startServer(next);
            } else {
                console.error(`No available ports between ${START_PORT}-${MAX_PORT}.`);
                process.exit(1);
            }
        } else {
            console.error('HTTP server error:', err);
        }
    });

    server.listen(port, () => {
        const wss = new WebSocketServer({ server });

        wss.on('connection', (ws) => {
            console.log('Client connected');

            ws.on('message', (message) => {
                // Broadcast the received message to all connected clients.
                wss.clients.forEach((client) => {
                    if (client.readyState === WebSocket.OPEN) {
                        client.send(message);
                    }
                });
            });

            ws.on('close', () => {
                console.log('Client disconnected');
            });

            ws.on('error', (error) => {
                console.error('WebSocket error:', error);
            });
        });

        console.log(`WebSocket server is running on ws://localhost:${port}`);
    });
}

startServer(START_PORT);
