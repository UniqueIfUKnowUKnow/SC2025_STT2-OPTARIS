import websocket
import json
import time
import os

# --- Configuration ---
# Set the WebSocket server URL.
# IMPORTANT: Replace 'localhost' with the actual IP address of the computer running the Node.js server.
# For example: 'ws://192.168.1.100:8080'
# Try multiple WS ports to match the GUI server's fallback behavior
WS_URLS = [
    "ws://localhost:8080",
    "ws://localhost:8081",
    "ws://localhost:8082",
    "ws://localhost:8083",
]

# --- Main Logic ---
def on_open(ws):
    """
    Called when the WebSocket connection is successfully established.
    Starts sending data in a loop.
    """
    print("Connection opened to WebSocket server.")

    # Continuously read the latest detected point written by firmware and forward to WS
    try:
        while True:
            # Determine path to shared JSON written by firmware
            base_dir = os.path.dirname(os.path.abspath(__file__))
            last_point_path = os.path.join(base_dir, 'last_point.json')

            # Default: if file missing or unreadable, skip sending this tick
            data_point = None
            if os.path.exists(last_point_path):
                try:
                    with open(last_point_path, 'r') as f:
                        payload = json.load(f)
                        # Ensure required fields exist
                        if all(k in payload for k in ("x", "y", "z", "timestamp")):
                            data_point = {
                                "x": float(payload["x"]),
                                "y": float(payload["y"]),
                                "z": float(payload["z"]),
                                "timestamp": int(payload["timestamp"])
                            }
                except Exception as e:
                    print(f"Failed reading last_point.json: {e}")

            if data_point is not None:
                json_data = json.dumps(data_point)
                try:
                    ws.send(json_data)
                    print(f"Sent data: {json_data}")
                except Exception as send_err:
                    print(f"WebSocket send failed: {send_err}")
            else:
                print("No last_point.json yet; waiting...")

            # Wait for 1 second before next check
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

    # Try a list of candidate WS URLs until one connects
    for url in WS_URLS:
        print(f"Attempting to connect to {url}...")
        try:
            ws_app = websocket.WebSocketApp(
                url,
                on_open=on_open,
                on_error=on_error,
                on_close=on_close
            )
            ws_app.run_forever()
            break
        except KeyboardInterrupt:
            raise
        except Exception as e:
            print(f"Connection failed for {url}: {e}")
