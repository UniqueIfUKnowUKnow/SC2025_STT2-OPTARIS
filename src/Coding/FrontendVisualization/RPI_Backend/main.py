# -*- coding: utf-8 -*-

# main.py (Reverted to simple Start Button version - CORRECTED)

# These are "import" statements.
# They tell Python we want to use pre-built code modules so we don't have to write everything from scratch.
import asyncio     # Lets us run multiple things at once without blocking (used for server + simulation running together)
import json        # Used to convert Python data into JSON text, and back again (for sending data to web clients)
import random      # Lets us create random numbers (used to make simulation more realistic)
import time        # Lets us work with time - get current time, measure durations, pause, etc.
import math        # Gives access to math functions (like sine, cosine, square root)
import argparse    # Lets us read special "command line arguments" when starting the program
from collections import deque  # deque is like a list but faster for adding/removing items from ends

import websockets  # A library for real-time communication between server and web browsers

# --- Configuration ---
HOST = "0.0.0.0"  # This means "listen for connections on all available network interfaces"
PORT = 8765       # The network port number clients will connect to
POSITION_HISTORY_MAX_LENGTH = 50  # We'll remember only the last 50 position points

# Scanner simulation configuration
SCANNER_MAX_RANGE_METERS = 12.0  # The maximum visible range of the scanner beam
SCANNER_FIXED_ELEVATION_Y = 1.0  # Fixed height for background and searching scans
SCAN_UPDATE_INTERVAL_SEC = 0.05  # 20 Hz updates for smooth motion in non-tracking phases
TRACKING_UPDATE_INTERVAL_SEC = 0.01  # 100 Hz updates during tracking for dense XYZ points
BACKGROUND_SWEEP_RPS = 0.25      # Background sweep revolutions per second (90 deg/sec)
SEARCH_SECTOR_HALF_ANGLE_RAD = math.radians(30.0)  # +/- 30 degrees sector in front
SEARCH_RASTER_PERIOD_SEC = 3.0   # Time for one full zig-zag sweep across the sector

# --- State Management ---
connected_clients = set()  # Keeps track of all clients currently connected to the server

# This dictionary keeps ALL information about the current state of the application.
# It will be sent to clients so they can know what's going on.
app_state = {
    "status": "STANDBY",    # What the system is currently doing
    "progress": 0,          # How far along a task is (0–100%)
    "warning": None,        # Any warning messages to show the user
    "error": None,          # Any error messages to show the user
    "payload": {            # Extra data - usually sensor readings, telemetry, and results
        "measured_pos": [None, None, None],   # Latest measured position [x, y, z]
        "scanner_pos": [None, None, None],    # Where the scanner is currently pointing [x, y, z]
        "live_telemetry": {},                 # Latest real-time measurements like speed, distance
        "position_history": [],               # List of previous positions
        "predicted_orbit_params": None,       # (legacy) kept for compatibility; frontend will compute prediction
        "initial_orbit_params": None,         # Orbit derived from TLE at tracking start
        "initial_tle": None                   # TLE lines provided at tracking start
    }
}

# deque is like a "rolling list" where old items drop off when we reach the max length
position_history_deque = deque(maxlen=POSITION_HISTORY_MAX_LENGTH)

# Remember the last known target position so that when signal is lost, the scanner can keep pointing there
last_known_target_pos = [None, None, None]

# Control flag that lets us request a simulation reset from a client command
reset_requested = False

# This is like a "trigger switch" that starts off unset.
# When the user sends the "start" command, we set it - and the simulation begins.
simulation_started = asyncio.Event()

# Add a pause event for backend pause/resume
pause_event = asyncio.Event()
pause_event.set()  # Start as unpaused

# --- WebSocket Server Logic ---
async def register(websocket):
    """
    Called whenever a new client connects.
    Adds them to our list of connected clients and sends them the current state.
    """
    connected_clients.add(websocket)
    print(f"Client connected: {websocket.remote_address}. Total clients: {len(connected_clients)}")
    try:
        # Send the current state so the new client knows what's happening right away
        await websocket.send(json.dumps(app_state))
    except websockets.ConnectionClosed:
        pass  # If the connection closes before we send, just ignore it

async def unregister(websocket):
    """
    Called whenever a client disconnects.
    Removes them from our list of connected clients.
    """
    if websocket in connected_clients:
        connected_clients.remove(websocket)
    print(f"Client disconnected: {websocket.remote_address}. Total clients: {len(connected_clients)}")

async def broadcast_state():
    """
    Sends the current application state to EVERY connected client.
    Now resilient to clients disconnecting between frames.
    """
    if not connected_clients:
        return  # No clients to send to
    message = json.dumps(app_state)  # Serialize the current state once
    # Work on a static list snapshot to avoid mutation during iteration
    clients_snapshot = list(connected_clients)
    # Keep track of clients that fail to receive, so we can remove them
    to_remove = []
    for client in clients_snapshot:
        try:
            await client.send(message)
        except websockets.ConnectionClosed:
            # If the client closed the connection, mark it for removal
            to_remove.append(client)
        except Exception as e:
            # Any other send error: also mark for removal to keep loop healthy
            to_remove.append(client)
    # Unregister failed clients to prevent future send errors
    for client in to_remove:
        try:
            await unregister(client)
        except Exception:
            # As a fallback, ensure the client is discarded from the set
            connected_clients.discard(client)

async def server_handler(websocket):
    """
    Handles each client's incoming messages.
    """
    global reset_requested
    await register(websocket)  # First, mark them as connected
    try:
        # Listen for messages from this client
        async for message in websocket:
            try:
                data = json.loads(message)  # Turn incoming JSON into Python data
                cmd = data.get("command")
                if cmd == "start":
                    print("Received start command. Triggering simulation or resuming if paused.")
                    simulation_started.set()  # This will let run_simulation() start
                    pause_event.set()  # Resume if paused
                elif cmd == "reset":
                    print("Received reset command. Requesting simulation reset.")
                    reset_requested = True  # Signal the simulation loop to restart
                elif cmd == "stop":
                    print("Received stop command. Pausing simulation loop.")
                    pause_event.clear()  # Pause the simulation loop
                # Note: Stop/pause is now handled on the backend as well
            except json.JSONDecodeError:
                print(f"Received invalid JSON: {message}")
    except websockets.ConnectionClosed:
        pass  # Ignore if connection closes unexpectedly
    finally:
        await unregister(websocket)  # Clean up

# --- Simulation Helpers ---

def set_scanner_pos(point_xyz):
    """Helper to update scanner position in the shared state."""
    app_state["payload"]["scanner_pos"] = [
        None if v is None else (round(float(v), 2)) for v in point_xyz
    ]

def compute_background_sweep(now_s):
    """Compute a 360-degree sweep around the origin at fixed elevation."""
    angle_rad = 2.0 * math.pi * BACKGROUND_SWEEP_RPS * now_s
    x = SCANNER_MAX_RANGE_METERS * math.cos(angle_rad)
    z = SCANNER_MAX_RANGE_METERS * math.sin(angle_rad)
    y = SCANNER_FIXED_ELEVATION_Y
    return [x, y, z]

def compute_searching_raster(now_s):
    """Compute a zig-zag (raster) scan in a +/- sector in front of the tracker."""
    # Azimuth sweeps back and forth between -sector and +sector as a triangle wave
    # Triangle wave in [-1, 1]
    phase = (now_s % SEARCH_RASTER_PERIOD_SEC) / SEARCH_RASTER_PERIOD_SEC
    tri = 4.0 * abs(phase - 0.5) - 1.0  # triangle wave in [-1, 1]
    azimuth = tri * SEARCH_SECTOR_HALF_ANGLE_RAD  # in radians
    # Keep elevation fixed and range at max range
    x = SCANNER_MAX_RANGE_METERS * math.cos(azimuth)
    z = SCANNER_MAX_RANGE_METERS * math.sin(azimuth)
    y = SCANNER_FIXED_ELEVATION_Y
    return [x, y, z]

def reset_simulation_state():
    global reset_requested, position_history_deque, last_known_target_pos
    reset_requested = False
    position_history_deque.clear()
    last_known_target_pos = [None, None, None]
    update_state(status="STANDBY", progress=0, warning=None, error=None,
                 payload_data={"measured_pos": [None, None, None], "position_history": [], "scanner_pos": [None, None, None], "initial_orbit_params": None, "initial_tle": None})

# --- Simulation Logic ---
def update_state(status=None, progress=None, warning=None, error=None, payload_data=None):
    """
    Safely updates the global application state.
    Only changes the parts we provide - leaves others untouched.
    """
    if status is not None:
        app_state["status"] = status
    if progress is not None:
        app_state["progress"] = progress
    if warning is not None:
        app_state["warning"] = warning
    if error is not None:
        app_state["error"] = error
    if payload_data is not None:
        app_state["payload"].update(payload_data)  # Merge payload updates

async def run_simulation():
    """
    Main simulation loop - runs fake 'hardware' actions and sends updates to clients.
    """
    await simulation_started.wait()  # Wait until the "start" button is pressed
    print("Simulation loop started.")
    
    # Define parameters for the fake orbit path
    orbit_a, orbit_b, orbit_speed, altitude = 8, 5, 0.5, 4
    start_time = time.time()  # Remember the starting time of the whole simulation

    global last_known_target_pos, reset_requested

    while True:  # This will loop forever
        # Step 1: Initialize hardware
        update_state(status="INITIALIZING_HARDWARE")
        await broadcast_state()
        for _ in range(int(2.0 / SCAN_UPDATE_INTERVAL_SEC)):
            if reset_requested:
                break
            await pause_event.wait()  # Pause if needed
            await asyncio.sleep(SCAN_UPDATE_INTERVAL_SEC)
        if reset_requested:
            reset_simulation_state()
            await broadcast_state()
            start_time = time.time()
            continue

        # Step 2: Start background scan
        update_state(status="BACKGROUND_SCAN_COMMENCING", progress=0)
        await broadcast_state()
        for _ in range(int(1.0 / SCAN_UPDATE_INTERVAL_SEC)):
            if reset_requested:
                break
            await pause_event.wait()
            await asyncio.sleep(SCAN_UPDATE_INTERVAL_SEC)
        if reset_requested:
            reset_simulation_state()
            await broadcast_state()
            start_time = time.time()
            continue

        # Step 3: Simulate scan progress from 0% to 100% with continuous 360 sweep
        update_state(status="BACKGROUND_SCAN_IN_PROGRESS")
        i = 0
        while i <= 100:
            if reset_requested:
                break
            await pause_event.wait()
            now_s = time.time() - start_time
            # Update scanner position for 360 sweep
            set_scanner_pos(compute_background_sweep(now_s))
            update_state(progress=i)
            await broadcast_state()
            await asyncio.sleep(SCAN_UPDATE_INTERVAL_SEC)  # 20 Hz updates
            i += 1
        if reset_requested:
            reset_simulation_state()
            await broadcast_state()
            start_time = time.time()
            continue

        # Step 4: Scan complete
        update_state(status="BACKGROUND_SCAN_COMPLETE", progress=100)
        await broadcast_state()
        for _ in range(int(1.0 / SCAN_UPDATE_INTERVAL_SEC)):
            if reset_requested:
                break
            await pause_event.wait()
            await asyncio.sleep(SCAN_UPDATE_INTERVAL_SEC)
        if reset_requested:
            reset_simulation_state()
            await broadcast_state()
            start_time = time.time()
            continue
        
        # Step 5: Start searching for target (scanner sweeps in front sector)
        update_state(
            status="SEARCHING_FOR_TARGET",
            error=None,
            warning=None,
            payload_data={
                "measured_pos": [None, None, None],
                "predicted_orbit_params": None,
            },
        )
        position_history_deque.clear()  # Clear past position history
        update_state(payload_data={"position_history": list(position_history_deque)})
        await broadcast_state()

        # Perform searching with raster sweep for 10 seconds
        searching_start = time.time()
        while time.time() - searching_start < 10.0:
            if reset_requested:
                break
            await pause_event.wait()
            now_s = time.time() - start_time
            set_scanner_pos(compute_searching_raster(now_s))
            await broadcast_state()
            await asyncio.sleep(SCAN_UPDATE_INTERVAL_SEC)
        if reset_requested:
            reset_simulation_state()
            await broadcast_state()
            start_time = time.time()
            continue

        # Step 6: Target found
        update_state(status="TARGET_ACQUIRED")
        await broadcast_state()
        for _ in range(int(1.0 / SCAN_UPDATE_INTERVAL_SEC)):
            if reset_requested:
                break
            await pause_event.wait()
            await asyncio.sleep(SCAN_UPDATE_INTERVAL_SEC)
        if reset_requested:
            reset_simulation_state()
            await broadcast_state()
            start_time = time.time()
            continue

        # Step 7: Begin tracking target with TLE and initial orbit parameters
        tle = {
            "line1": "1 25544U 98067A   20344.91667824  .00001264  00000-0  29621-4 0  9993",
            "line2": "2 25544  51.6441 132.7192 0002275 214.2790 212.8861 15.49312212256555"
        }
        initial_orbit_params = {"type": "ellipse", "a": orbit_a, "b": orbit_b, "altitude": altitude}
        update_state(status="TRACKING_TARGET", warning=None, error=None, payload_data={
            "predicted_orbit_params": None,  # frontend will compute live prediction
            "initial_orbit_params": initial_orbit_params,
            "initial_tle": tle
        })
        
        tracking_start_time = time.time()  # Mark when tracking began
        
        # Track target for 20 seconds
        while time.time() - tracking_start_time < 20:
            if reset_requested:
                break
            await pause_event.wait()
            current_time = time.time()
            angle = (current_time - start_time) * orbit_speed  # How far around the orbit we are
            # Generate x, y, z position with slight randomness
            x = orbit_a * math.cos(angle) + random.uniform(-0.1, 0.1)
            y = altitude + random.uniform(-0.1, 0.1)
            z = orbit_b * math.sin(angle) + random.uniform(-0.1, 0.1)
            pos = [round(x, 2), round(y, 2), round(z, 2)]
            position_history_deque.append(pos)  # Remember this position
            last_known_target_pos = pos[:]  # Update last known target position
            
            # Create fake telemetry data
            telemetry = {
                "distance": round(math.sqrt(x**2 + y**2 + z**2), 2),
                "signal_strength": random.randint(85, 99),
                "pan_angle": round(math.degrees(math.atan2(z, x)), 1),
                "tilt_angle": round(math.degrees(math.asin(y / math.sqrt(x**2 + y**2 + z**2))), 1)
            }
            # During tracking, scanner_pos locks to measured_pos
            set_scanner_pos(pos)
            update_state(payload_data={"measured_pos": pos, "position_history": list(position_history_deque), "live_telemetry": telemetry})
            await broadcast_state()
            await asyncio.sleep(TRACKING_UPDATE_INTERVAL_SEC)  # 100 Hz updates during tracking
        if reset_requested:
            reset_simulation_state()
            await broadcast_state()
            start_time = time.time()
            continue
            
        # Step 8: LIDAR warning
        update_state(warning="LOW_LIDAR_SIGNAL_STRENGTH")
        await broadcast_state()
        for _ in range(int(4.0 / SCAN_UPDATE_INTERVAL_SEC)):
            if reset_requested:
                break
            await pause_event.wait()
            await asyncio.sleep(SCAN_UPDATE_INTERVAL_SEC)
        if reset_requested:
            reset_simulation_state()
            await broadcast_state()
            start_time = time.time()
            continue

        # Step 9: Target lost - keep scanner pointed at last known target position
        update_state(status="TARGET_SIGNAL_LOST", payload_data={"measured_pos": [None, None, None]})
        loss_start = time.time()
        while time.time() - loss_start < 5.0:
            if reset_requested:
                break
            await pause_event.wait()
            # Keep scanner locked at last known position
            set_scanner_pos(last_known_target_pos if all(v is not None for v in last_known_target_pos) else [None, None, None])
            await broadcast_state()
            await asyncio.sleep(SCAN_UPDATE_INTERVAL_SEC)
        if reset_requested:
            reset_simulation_state()
            await broadcast_state()
            start_time = time.time()
            continue

        # Step 10: Resume searching with error flag
        update_state(status="SEARCHING_FOR_TARGET", error="ERROR_TARGET_LOST")
        await broadcast_state()
        for _ in range(int(5.0 / SCAN_UPDATE_INTERVAL_SEC)):
            if reset_requested:
                break
            await pause_event.wait()
            await asyncio.sleep(SCAN_UPDATE_INTERVAL_SEC)
        if reset_requested:
            reset_simulation_state()
            await broadcast_state()
            start_time = time.time()
            continue

async def main(simulate: bool):
    """
    Entry point for the program.
    If simulate=True, starts the WebSocket server and the simulation.
    """
    if not simulate:
        print("Run with --simulate flag.")
        return
    # Start WebSocket server
    server = await websockets.serve(server_handler, HOST, PORT)
    print(f"WebSocket server started on ws://{HOST}:{PORT}. Waiting for client to send 'start' command.")
    # Start simulation in background
    simulation_task = asyncio.create_task(run_simulation())
    # Wait for both server and simulation to finish (which is basically forever)
    await asyncio.gather(simulation_task, server.wait_closed())

if __name__ == "__main__":
    # This only runs if we start the script directly (not if imported from somewhere else)
    parser = argparse.ArgumentParser(description="Drone Tracker Backend")
    parser.add_argument('--simulate', action='store_true', help='Run in simulation mode.')
    args = parser.parse_args()
    try:
        asyncio.run(main(args.simulate))
    except KeyboardInterrupt:
        print("\nShutting down server.")
