# ================================================================= 
# Python Scanner with Distance Calibration and Initial Scan 
# 
# Purpose: This script first performs a calibration phase to find an 
# average background distance, then enters a scanning phase to 
# detect any object that is significantly closer. 
# 
# NEW: This script now includes a WebSocket server to broadcast real-time 
# data to connected frontend applications, and command-line arguments 
# to control which stages to run.
# ================================================================= 

# --- Standard Library Imports --- 
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins 
import pigpio            # For stable, hardware-based PWM for the servo 
import time              # For creating delays 
import serial            # For reading data from the LiDAR's serial port 
import threading         # To run the LiDAR reader in the background 
import queue             # For thread-safe data sharing between threads 
import statistics        # For easily calculating the average distance 
import asyncio          # NEW: For handling WebSocket connections asynchronously
import websockets       # NEW: For creating WebSocket server to communicate with frontend
import json             # NEW: For formatting data as JSON messages
import argparse         # NEW: For handling command-line arguments
import signal           # NEW: For graceful shutdown handling

# --- Pin & Port Configuration (BCM numbering) --- 
# Stepper Motor Pins 
DIR_PIN    = 26 
STEP_PIN   = 13 
ENABLE_PIN = 23 
RESET_PIN  = 19 
M0_PIN     = 0 
M1_PIN     = 5 
M2_PIN     = 6 

# Servo Motor Pin 
SERVO_PIN  = 2 

# LiDAR Serial Port (GPIO 14/15 -> /dev/ttyS0) 
LIDAR_SERIAL_PORT = '/dev/ttyS0' 
LIDAR_BAUD_RATE = 115200 

# --- WebSocket Configuration ---
# NEW: These settings control the WebSocket server that broadcasts data to the frontend
WEBSOCKET_HOST = '0.0.0.0'  # Listen on all network interfaces (allows external connections)
WEBSOCKET_PORT = 8080        # Port number for WebSocket connections
WEBSOCKET_CLIENTS = set()    # NEW: Set to store all connected frontend clients

# --- Motor & Movement Settings --- 
STEPS_PER_REVOLUTION = 6400 
STEPPER_PULSE_DELAY = 0.0001 
MIN_PULSE_WIDTH = 500 
MAX_PULSE_WIDTH = 2500 

# --- Stepper Sweep Settings --- 
SWEEP_RANGE_DEGREES = 40 
STEPS_FOR_SWEEP = int((SWEEP_RANGE_DEGREES / 360.0) * STEPS_PER_REVOLUTION) 

# --- Calibration & Detection Settings --- 
CALIBRATION_SWEEPS = 2 
# An object is "significant" if it's 20% closer than the average. 
DETECTION_THRESHOLD_FACTOR = 0.8 

# --- Global Variables for WebSocket Broadcasting ---
# NEW: These variables store the current state and data that we broadcast to connected frontends
current_system_state = "INITIALIZING"  # Current state of the system (CALIBRATING, SCANNING, DETECTED, FINISHED)
current_progress = 0                   # Progress percentage (0-100)
current_message = ""                   # Human-readable status message
current_data = {}                      # Additional data specific to current state

# --- WebSocket Server Functions ---
# NEW: These functions handle the WebSocket server that broadcasts data to connected frontends

async def register_client(websocket, path):
    """
    NEW: This function is called whenever a frontend connects to our WebSocket server.
    It adds the new connection to our list of clients and sends them the current state.
    """
    print(f"Frontend connected from {websocket.remote_address}")  # Log the new connection
    WEBSOCKET_CLIENTS.add(websocket)  # Add this connection to our list of clients
    
    try:
        # Send the current state immediately to the new client
        await send_state_update(websocket)
        
        # Keep the connection alive and wait for any messages from the frontend
        async for message in websocket:
            # Currently we don't handle incoming messages, but this keeps the connection open
            pass
    except websockets.exceptions.ConnectionClosed:
        # Connection was closed by the frontend
        pass
    finally:
        # Remove the client from our list when they disconnect
        WEBSOCKET_CLIENTS.remove(websocket)
        print(f"Frontend disconnected from {websocket.remote_address}")

async def broadcast_to_all_clients(message):
    """
    NEW: This function sends a message to all connected frontend clients.
    It's used to broadcast real-time updates about the scanning process.
    """
    if WEBSOCKET_CLIENTS:  # Only send if we have connected clients
        # Create a copy of the clients set to avoid modification during iteration
        clients_to_notify = WEBSOCKET_CLIENTS.copy()
        
        # Send the message to each connected client
        for client in clients_to_notify:
            try:
                await client.send(json.dumps(message))  # Convert Python dict to JSON string
            except websockets.exceptions.ConnectionClosed:
                # Remove disconnected clients
                WEBSOCKET_CLIENTS.discard(client)
            except Exception as e:
                print(f"Error sending message to client: {e}")
                WEBSOCKET_CLIENTS.discard(client)

async def send_state_update(websocket):
    """
    NEW: This function sends the current system state to a specific client.
    It's used when a new client connects to give them the current status.
    """
    try:
        message = {
            "state": current_system_state,
            "payload": {
                "progress": current_progress,
                "message": current_message,
                **current_data  # Include any additional data for the current state
            }
        }
        await websocket.send(json.dumps(message))  # Send as JSON
    except Exception as e:
        print(f"Error sending state update: {e}")

def update_system_state(state, progress, message, data=None):
    """
    NEW: This function updates the global system state and broadcasts the change to all connected frontends.
    It's called whenever something important happens in the scanning process.
    """
    global current_system_state, current_progress, current_message, current_data
    
    # Update our global variables
    current_system_state = state
    current_progress = progress
    current_message = message
    current_data = data or {}
    
    # Create the message to send to frontends
    message_to_send = {
        "state": state,
        "payload": {
            "progress": progress,
            "message": message,
            **current_data
        }
    }
    
    # Broadcast to all connected clients (run in background)
    asyncio.create_task(broadcast_to_all_clients(message_to_send))
    
    # Also print to console for debugging
    print(f"State Update: {state} - {message}")

async def start_websocket_server():
    """
    NEW: This function starts the WebSocket server that allows frontends to connect.
    It runs in the background and waits for connections from the web application.
    """
    try:
        # Start the WebSocket server
        server = await websockets.serve(register_client, WEBSOCKET_HOST, WEBSOCKET_PORT)
        print(f"WebSocket server started on ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
        print("Waiting for frontend connections...")
        
        # Keep the server running
        await server.wait_closed()
    except Exception as e:
        print(f"Error starting WebSocket server: {e}")

# --- LiDAR Reader Thread --- 
class LidarReader(threading.Thread): 
    """ 
    A dedicated thread that continuously reads data from the TFmini-S LiDAR. 
    """ 
    def __init__(self, port, baudrate, data_queue): 
        super().__init__(daemon=True) 
        self.port = port 
        self.baudrate = baudrate 
        self.data_queue = data_queue 
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1) 
        self.ser.flushInput() 
        self.frame_header = 0x59 
        print("LiDAR Reader thread initialized.") 

    def run(self): 
        while True: 
            if self.ser.read(1) == b'\x59': 
                if self.ser.read(1) == b'\x59': 
                    frame = b'\x59\x59' + self.ser.read(7) 
                    if len(frame) == 9: 
                        checksum = sum(frame[:-1]) & 0xFF 
                        if checksum == frame[8]: 
                            distance_cm = frame[2] + (frame[3] << 8) 
                            if distance_cm > 0: 
                                self.data_queue.put(distance_cm) 

# --- Setup and Control Functions --- 
def setup_stepper_gpio(): 
    """ 
    Configures the RPi's GPIO pins for controlling the DRV8825 stepper driver. 
    """ 
    GPIO.setwarnings(False) 
    GPIO.setmode(GPIO.BCM) 
    pins = [DIR_PIN, STEP_PIN, ENABLE_PIN, RESET_PIN, M0_PIN, M1_PIN, M2_PIN] 
    for pin in pins: 
        GPIO.setup(pin, GPIO.OUT) 
    GPIO.output(ENABLE_PIN, GPIO.LOW) 
    GPIO.output(RESET_PIN, GPIO.HIGH) 
    GPIO.output(M0_PIN, GPIO.HIGH) 
    GPIO.output(M1_PIN, GPIO.HIGH) 
    GPIO.output(M2_PIN, GPIO.HIGH) 

def set_servo_angle(pi, angle): 
    """ 
    Calculates the required pulse width and sets the servo to a specific angle. 
    """ 
    pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH) 
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width) 

# --- Main Application --- 
async def main_async(mode, stage=None): 
    """
    NEW: This is the main function that runs the scanning process.
    It's now async to work with the WebSocket server and includes mode/stage control.
    """
    global current_system_state, current_progress, current_message, current_data
    
    # --- Setup --- 
    setup_stepper_gpio() 
    pi = pigpio.pi() 
    if not pi.connected: 
        print("Could not connect to pigpio daemon. Is it running?") 
        return 

    lidar_data_queue = queue.Queue() 
    lidar_thread = LidarReader(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, lidar_data_queue) 
    lidar_thread.start() 

    # --- State Machine and Variables --- 
    states = ["CALIBRATING", "SCANNING", "DETECTED"] 
    current_state = states[0] 
    
    # NEW: Update the system state and broadcast to frontends
    update_system_state("CALIBRATING", 0, "Starting calibration phase...")
    print(f"Current State: {current_state}") 

    stepper_steps_taken = 0 
    stepper_direction_cw = True 
    GPIO.output(DIR_PIN, GPIO.HIGH) 
     
    constant_servo_angle = 20 
    set_servo_angle(pi, constant_servo_angle) 
    print(f"Servo angle set to {constant_servo_angle} degrees.") 

    calibration_distances = [] 
    sweeps_completed = 0 
    average_distance = 0 

    try: 
        # ================================================================= 
        # 1. CALIBRATION PHASE 
        # ================================================================= 
        if stage is None or stage == "calibrate":
            while current_state == "CALIBRATING": 
                GPIO.output(STEP_PIN, GPIO.HIGH) 
                time.sleep(STEPPER_PULSE_DELAY) 
                GPIO.output(STEP_PIN, GPIO.LOW) 
                time.sleep(STEPPER_PULSE_DELAY) 
                stepper_steps_taken += 1 

                try: 
                    distance = lidar_data_queue.get_nowait() 
                    calibration_distances.append(distance) 
                except queue.Empty: 
                    pass 

                if stepper_steps_taken >= STEPS_FOR_SWEEP: 
                    stepper_steps_taken = 0 
                    stepper_direction_cw = not stepper_direction_cw 
                    GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW) 
                    sweeps_completed += 0.5 
                    
                    # NEW: Calculate progress percentage and broadcast update
                    progress_percent = int((sweeps_completed / CALIBRATION_SWEEPS) * 100)
                    message = f"Calibration sweep {int(sweeps_completed * 2)} of {CALIBRATION_SWEEPS * 2} halfs completed..."
                    update_system_state("CALIBRATING", progress_percent, message)
                    
                    print(f"Calibration sweep {int(sweeps_completed * 2)} of {CALIBRATION_SWEEPS * 2} halfs completed...") 

                    if sweeps_completed >= CALIBRATION_SWEEPS: 
                        if calibration_distances: 
                            average_distance = statistics.mean(calibration_distances) 
                            
                            # NEW: Broadcast calibration completion with final results
                            update_system_state("CALIBRATING", 100, "Calibration complete!", {
                                "status": "complete",
                                "average_distance": round(average_distance, 2)
                            })
                            
                            print("\n" + "="*30) 
                            print("CALIBRATION COMPLETE") 
                            print(f"Average Background Distance: {average_distance:.2f} cm") 
                            print("="*30 + "\n") 
                            
                            if stage == "calibrate":
                                # If only running calibration stage, exit here
                                update_system_state("FINISHED", 100, "Calibration stage completed successfully")
                                return
                            
                            current_state = states[1] # Transition to the next state 
                            update_system_state("SCANNING", 0, "Starting scanning phase...")
                            print(f"Current State: {current_state}") 
                        else: 
                            print("Calibration Failed: No LiDAR data collected.") 
                            update_system_state("FINISHED", 0, "Calibration failed: No LiDAR data collected")
                            return 

        # Reset stepper state for the next phase 
        stepper_steps_taken = 0 
         
        # ================================================================= 
        # 2. INITIAL SCAN PHASE 
        # ================================================================= 
        if stage is None or stage == "scan":
            while current_state == "SCANNING": 
                GPIO.output(STEP_PIN, GPIO.HIGH) 
                time.sleep(STEPPER_PULSE_DELAY) 
                GPIO.output(STEP_PIN, GPIO.LOW) 
                time.sleep(STEPPER_PULSE_DELAY) 
                stepper_steps_taken += 1 

                # --- Detection Logic --- 
                try: 
                    distance = lidar_data_queue.get_nowait() 
                     
                    # Compare current distance to the calibrated average 
                    if distance < (average_distance * DETECTION_THRESHOLD_FACTOR): 
                        # Calculate the stepper's current angle within the sweep 
                        # Note: This is a simplified angle relative to the sweep start. 
                        # 160 is the starting angle of the 40-degree sweep (180 - 40/2). 
                        angle_offset = (stepper_steps_taken / STEPS_FOR_SWEEP) * SWEEP_RANGE_DEGREES 
                        current_stepper_angle = 160 + angle_offset if stepper_direction_cw else 200 - angle_offset 

                        # NEW: Broadcast target detection with final results
                        update_system_state("DETECTED", 100, "Target detected!", {
                            "distance": distance,
                            "stepper_angle": round(current_stepper_angle, 1),
                            "servo_angle": constant_servo_angle
                        })

                        print("\n" + "="*40) 
                        print(f"TARGET DETECTED!") 
                        print(f"  -> Distance: {distance} cm (Average was {average_distance:.2f} cm)") 
                        print(f"  -> Stepper Angle: {current_stepper_angle:.1f}°") 
                        print(f"  -> Servo Angle: {constant_servo_angle}°") 
                        print("="*40 + "\n") 
                        current_state = states[2] # Change state to stop the scan 
                         
                except queue.Empty: 
                    pass 

                # NEW: Broadcast real-time scanning data for live visualization
                if stepper_steps_taken % 10 == 0:  # Send update every 10 steps to avoid spam
                    angle_offset = (stepper_steps_taken / STEPS_FOR_SWEEP) * SWEEP_RANGE_DEGREES 
                    current_stepper_angle = 160 + angle_offset if stepper_direction_cw else 200 - angle_offset 
                    
                    # Calculate progress percentage for scanning phase
                    scan_progress = int((stepper_steps_taken / STEPS_FOR_SWEEP) * 100)
                    
                    update_system_state("SCANNING", scan_progress, "Scanning in progress...", {
                        "angle": round(current_stepper_angle, 1),
                        "distance": distance if 'distance' in locals() else None
                    })

                # Reverse direction at the end of the sweep 
                if stepper_steps_taken >= STEPS_FOR_SWEEP: 
                    stepper_steps_taken = 0 
                    stepper_direction_cw = not stepper_direction_cw 
                    GPIO.output(DIR_PIN, GPIO.HIGH if stepper_direction_cw else GPIO.LOW) 

            print("Scanning complete.") 
            
            # NEW: If only running scan stage, mark as finished
            if stage == "scan":
                update_system_state("FINISHED", 100, "Scanning stage completed successfully")

    except KeyboardInterrupt: 
        print("\nProgram stopped by user.") 
        update_system_state("FINISHED", 0, "Program stopped by user")
    finally: 
        print("Cleaning up...") 
        pi.set_servo_pulsewidth(SERVO_PIN, 0) 
        pi.stop() 
        GPIO.cleanup() 

def main():
    """
    NEW: Main entry point that handles command-line arguments and starts the appropriate mode.
    """
    # Create argument parser for command-line options
    parser = argparse.ArgumentParser(description='LiDAR Scanner with WebSocket Server')
    parser.add_argument('--mode', choices=['real', 'sim'], default='real',
                       help='Mode to run: real (with hardware) or sim (simulation only)')
    parser.add_argument('--stage', choices=['calibrate', 'scan'], 
                       help='Run only specific stage: calibrate or scan (optional)')
    
    # Parse the command-line arguments
    args = parser.parse_args()
    
    # Check if we're in real mode (hardware mode)
    if args.mode == 'real':
        print(f"Starting LiDAR Scanner in REAL mode")
        if args.stage:
            print(f"Running only {args.stage} stage")
        else:
            print("Running full calibration and scanning sequence")
        
        # Create event loop for async operations
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            print("Starting WebSocket server...")
            # Start the WebSocket server
            loop.run_until_complete(start_websocket_server())
            
        except KeyboardInterrupt:
            print("\nShutting down gracefully...")
        finally:
            loop.close()
    else:
        print("Simulation mode selected - no hardware operations will be performed")
        print("Use --mode real to run with actual hardware")

if __name__ == '__main__': 
    main() 

 