// Import the WebSocketServer class from the 'ws' library.
import { WebSocketServer } from 'ws';

// Set the port for the WebSocket server to listen on.
const PORT = 8080;

// Create a new WebSocket server instance.
const wss = new WebSocketServer({ port: PORT });

// Set up a listener for new client connections.
wss.on('connection', ws => {
    console.log('Client connected');

    // Set up a listener for messages from the connected client.
    ws.on('message', message => {
        // Log the message received from the client for debugging purposes.
        console.log(`Received message: ${message}`);

        // Broadcast the received message to all connected clients.
        wss.clients.forEach(client => {
            // Check if the client is still open and ready to receive messages.
            if (client.readyState === WebSocketServer.OPEN) {
                // Send the message to the client.
                client.send(message);
            }
        });
    });

    // Set up a listener for when a client disconnects.
    wss.on('close', () => {
        console.log('Client disconnected');
    });

    // Set up a listener for errors.
    wss.on('error', error => {
        console.error('WebSocket error:', error);
    });
});

// Log a message to the console to let us know the server is running.
console.log(`WebSocket server is running on ws://localhost:${PORT}`);
