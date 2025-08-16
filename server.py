import asyncio
import websockets
import datetime

async def time_sender(websocket):
    """
    Sends the current UTC time to the WebSocket client every second.
    """
    try:
        while True:
            now = datetime.datetime.utcnow().isoformat() + "Z"
            await websocket.send(f"Current time: {now}")
            await asyncio.sleep(1)
    except websockets.exceptions.ConnectionClosed as e:
        print(f"Client disconnected: {e}")

async def main():
    """
    Main function to start the WebSocket server.
    """
    async with websockets.serve(time_sender, "0.0.0.0", 8765):
        print("WebSocket server started on ws://0.0.0.0:8765")
        await asyncio.Future()  # This will keep the server running forever

if __name__ == "__main__":
    asyncio.run(main())
















# # This Python script sets up a simple WebSocket server on the Raspberry Pi.
# # It requires the 'websockets' library, which you can install with:
# # pip install websockets

# import asyncio
# import websockets
# import datetime

# # This is the async function that handles a single WebSocket connection.
# async def time_server(websocket, path):
#     """
#     Handles a single WebSocket connection, sending the current time every 3 seconds.
#     """
#     print(f"Client connected from {websocket.remote_address}")
#     try:
#         while True:
#             # Generate a message to send to the client.
#             # You can replace this with any data you want to send,
#             # for example, sensor readings or status updates.
#             now = datetime.datetime.now().isoformat()
#             message = f"Hello from RPi! The current time is: {now}"

#             # Send the message to the connected client.
#             await websocket.send(message)

#             # Wait for 3 seconds before sending the next message.
#             await asyncio.sleep(3)

#     except websockets.exceptions.ConnectionClosed as e:
#         print(f"Connection closed by client: {e.reason} ({e.code})")
#     except Exception as e:
#         print(f"An error occurred: {e}")
#     finally:
#         print(f"Client from {websocket.remote_address} disconnected.")


# # This is the main function to start the server.
# async def main():
#     """
#     Starts the WebSocket server on a specific host and port.
#     """
#     # Use '0.0.0.0' to make the server accessible from other machines on the network.
#     # Replace '8765' with your desired port number.
#     host = '0.0.0.0'
#     port = 8765

#     # Start the server using the websockets library.
#     async with websockets.serve(time_server, host, port):
#         print(f"WebSocket server started on ws://{host}:{port}")
#         # Keep the server running indefinitely.
#         await asyncio.Future()

# # Run the main function to start the server.
# if __name__ == "__main__":
#     asyncio.run(main())




# import asyncio
# import websockets
# import datetime

# async def time_sender(websocket, path):
#     while True:
#         now = datetime.datetime.utcnow().isoformat() + "Z"
#         await websocket.send(f"Current time is: {now}")
#         await asyncio.sleep(1)

# start_server = websockets.serve(time_sender, "0.0.0.0", 8765)

# asyncio.get_event_loop().run_until_complete(start_server)
# asyncio.get_event_loop().run_forever()