import asyncio
import websockets
import json
import random
import datetime

# A simple counter for the data stream
counter = 0

# The main handler for each new WebSocket connection
async def data_handler(websocket, path):
    """
    Handles a new WebSocket connection. It sends a JSON object with
    dynamic data every 2 seconds.

    Args:
        websocket: The WebSocket connection object.
        path: The path requested by the client (unused in this example).
    """
    global counter
    print(f"Client connected from {websocket.remote_address}")
    try:
        while True:
            # --- Easily change the data you want to send here ---
            # Create a dynamic data payload. You can replace this with
            # sensor readings, a status update, or any other data.
            data = {
                "id": counter,
                "value": random.uniform(0, 100),
                "timestamp": datetime.datetime.now().isoformat()
            }
            # Convert the Python dictionary to a JSON string
            message = json.dumps(data)
            
            # Send the message to the connected client
            await websocket.send(message)
            
            print(f"Sent: {message}")
            
            counter += 1
            # Wait for 2 seconds before sending the next message
            await asyncio.sleep(2)

    except websockets.exceptions.ConnectionClosed as e:
        # This block runs when the client closes the connection
        print(f"Client disconnected with code {e.code}: {e.reason}")
    finally:
        print("Connection handler finished.")

# The function to start the WebSocket server
async def start_server():
    """
    Starts the WebSocket server on IP '10.111.244.104' and port 8765.
    '0.0.0.0' makes the server accessible from any machine on the network.
    """
    RPi_IP = "10.111.244.104"
    print("Starting WebSocket server on ws://10.111.244.104:8765")
    async with websockets.serve(data_handler, RPi_IP, 8765):
        # The server will run forever until it's manually stopped
        await asyncio.Future()

if __name__ == "__main__":
    # To run this script, make sure you have the websockets library installed:
    # pip install websockets
    asyncio.run(start_server())

