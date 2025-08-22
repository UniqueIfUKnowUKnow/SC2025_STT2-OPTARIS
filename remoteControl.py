import asyncio
import websockets
import subprocess

# The IP address and port the Raspberry Pi will listen on for the server
HOST = '0.0.0.0'
PORT = 8770

# A list of approved commands that the client can execute
# You must add all allowed commands to this list
# For example, to move a servo, you might have a script named "move_servo.py"
# and the command would be "python /path/to/your/script/move_servo.py"
APPROVED_COMMANDS = [
    'echo "Hello from Raspberry Pi!"',
    'ls -l',
    'python /home/pi/my_project/camera_control.py'
    # Add your specific commands here
]

async def command_executor():
    while True:
        try:
            # The client connects to the server at the specified IP and port
            async with websockets.connect(f"ws://127.0.0.1:{PORT}") as websocket:
                print(f"Connected to WebSocket server on port {PORT}")
                while True:
                    # Receive a command string from the server
                    command_string = await websocket.recv()
                    print(f"Received command: '{command_string}'")

                    # Check if the received command is in the list of APPROVED_COMMANDS
                    if command_string in APPROVED_COMMANDS:
                        print("Command is approved. Executing...")
                        
                        # Use subprocess.run to execute the command in the shell
                        # The shell=True argument is necessary for commands with pipes or redirects
                        try:
                            result = subprocess.run(
                                command_string, 
                                shell=True, 
                                check=True, 
                                text=True, 
                                capture_output=True
                            )
                            print("Command executed successfully.")
                            print("Stdout:", result.stdout)
                            print("Stderr:", result.stderr)
                        except subprocess.CalledProcessError as e:
                            print(f"Command execution failed with error: {e}")
                            print("Stdout:", e.stdout)
                            print("Stderr:", e.stderr)
                            
                    else:
                        print(f"Warning: Command '{command_string}' is not in the approved list. Ignoring.")

        except websockets.exceptions.ConnectionClosedError:
            print(f"Connection to server closed. Reconnecting in 5 seconds...")
        except ConnectionRefusedError:
            print(f"Connection refused. Server may not be running. Retrying in 5 seconds...")
        except Exception as e:
            print(f"An unexpected error occurred: {e}. Retrying in 5 seconds...")
        
        await asyncio.sleep(5)  # Wait before trying to reconnect

if __name__ == "__main__":
    asyncio.run(command_executor())