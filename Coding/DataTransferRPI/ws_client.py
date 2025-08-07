import asyncio
import websockets

# This async function will connect to the server and exchange messages
async def send_and_receive():
    # IMPORTANT: Replace this with your Raspberry Pi's actual IP address
    # The port must match the server's port (8765)
    uri = "ws://192.168.55.180:8765" 

    try:
        async with websockets.connect(uri) as websocket:
            # The message we want to send to the server
            message_to_send = "Hello from your Windows PC!"

            print(f">>> Sending to server: {message_to_send}")
            await websocket.send(message_to_send)

            # Wait to receive the server's response
            response = await websocket.recv()
            print(f"<<< Received from server: {response}")

    except ConnectionRefusedError:
        print(f"Connection to {uri} failed. Is the server running on the Pi?")
    except Exception as e:
        print(f"An error occurred: {e}")

# Run the async function
if __name__ == "__main__":
    asyncio.run(send_and_receive())
