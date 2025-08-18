import websocket
import json
import time
import random

# --- Configuration ---
# Set the WebSocket server URL.
# IMPORTANT: Replace 'localhost' with the actual IP address of the computer running the Node.js server.
# For example: 'ws://192.168.1.100:8080'
WEBSOCKET_URL = "ws://localhost:8080"

# --- Main Logic ---
def on_open(ws):
    """
    Called when the WebSocket connection is successfully established.
    Starts sending data in a loop.
    """
    print("Connection opened to WebSocket server.")

    # A simple loop to simulate sending sensor data.
    # In a real-world scenario, you would read this data from your sensors.
    try:
        while True:
            # Generate a new point with a timestamp.
            # Here we use random values, but you would get your real sensor data here.
            data_point = {
                "x": random.uniform(-100, 100),
                "y": random.uniform(-100, 100),
                "z": random.uniform(-100, 100),
                "timestamp": int(time.time() * 1000)
            }

            # Convert the Python dictionary to a JSON string.
            json_data = json.dumps(data_point)

            # Send the JSON data through the WebSocket.
            ws.send(json_data)
            print(f"Sent data: {json_data}")

            # Wait for 1 second before sending the next data point.
            time.sleep(1)

    except KeyboardInterrupt:
        # Gracefully handle a keyboard interrupt (Ctrl+C).
        print("Stopping data transmission.")
        ws.close()
    except Exception as e:
        # Handle any other exceptions that might occur.
        print(f"An error occurred: {e}")
        ws.close()

def on_error(ws, error):
    """
    Called when a WebSocket error occurs.
    """
    print(f"WebSocket error: {error}")

def on_close(ws, close_status_code, close_msg):
    """
    Called when the WebSocket connection is closed.
    """
    print(f"Connection closed with status code: {close_status_code}, message: {close_msg}")

if __name__ == "__main__":
    # Enable debugging to see more verbose output.
    # websocket.enableTrace(True)

    # Create a new WebSocketApp instance with event handlers.
    ws_app = websocket.WebSocketApp(
        WEBSOCKET_URL,
        on_open=on_open,
        on_error=on_error,
        on_close=on_close
    )

    # Run the WebSocket application.
    # The `run_forever()` method will block and keep the script running.
    # It also handles automatic reconnections.
    ws_app.run_forever()
