#!/usr/bin/env python3
"""
Enhanced Drone Tracker Digital Twin Backend
===========================================

This script provides a comprehensive WebSocket server that simulates a satellite tracking
and LiDAR monitoring system. It manages the system's state machine, generates realistic
data, and communicates with the frontend React application.

Key Features:
- State machine management (INITIALIZING, BACKGROUND_SCAN, SEARCHING, TRACKING)
- Start/Stop/Reset command handling
- Real-time LiDAR simulation with pan/tilt angles
- Orbital parameter calculations
- WebSocket communication with the frontend

"""

import asyncio
import json
import math
import time
import random
import websockets
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Tuple
import argparse

# System Configuration Constants
# These values control how the system behaves and what data it generates

# WebSocket server configuration
WEBSOCKET_HOST = "0.0.0.0"  # Listen on all network interfaces
WEBSOCKET_PORT = 8765        # Port number for WebSocket connections

# LiDAR sensor configuration
LIDAR_MAX_RANGE = 12.0       # Maximum range in meters
LIDAR_MIN_RANGE = 0.1        # Minimum range in meters
LIDAR_ANGLE_RESOLUTION = 1.0 # Resolution in degrees

# Scanner movement configuration
SCAN_SPEED_DEG_PER_SEC = 30.0    # How fast the scanner moves during background scan
SEARCH_SPEED_DEG_PER_SEC = 45.0  # How fast the scanner moves during target search
TRACKING_UPDATE_RATE_HZ = 10.0   # How often tracking data is updated

# Simulation timing configuration
SIMULATION_UPDATE_RATE_HZ = 20.0  # How often the simulation updates
SIMULATION_STEP_TIME_MS = int(1000 / SIMULATION_UPDATE_RATE_HZ)  # Time between updates in milliseconds

# Orbital calculation configuration
MIN_POSITIONS_FOR_ORBIT = 10      # Minimum number of positions needed to calculate orbit
ORBIT_CALCULATION_INTERVAL_MS = 5000  # How often to recalculate orbital parameters

class SystemState:
    """
    System State Enumeration
    
    This defines all the possible states the system can be in.
    Each state represents a different mode of operation.
    """
    STANDBY = "STANDBY"                           # System is ready but not actively doing anything
    INITIALIZING = "INITIALIZING"                  # System is starting up and calibrating
    BACKGROUND_SCAN_IN_PROGRESS = "BACKGROUND_SCAN_IN_PROGRESS"  # Scanning the environment for targets
    SEARCHING_FOR_TARGET = "SEARCHING_FOR_TARGET"  # Actively looking for a specific target
    TRACKING_TARGET = "TRACKING_TARGET"            # Following and monitoring a target
    ERROR = "ERROR"                                # System has encountered an error
    PAUSED = "PAUSED"                             # System is temporarily stopped

class DroneTrackerSystem:
    """
    Main System Class
    
    This class manages the entire drone tracking system, including:
    - State machine logic
    - LiDAR simulation
    - Data generation
    - WebSocket communication
    """
    
    def __init__(self):
        """Initialize the system with default values"""
        
        # System state and control
        self.current_state = SystemState.STANDBY
        self.is_running = False
        self.simulation_start_time = None
        self.last_update_time = None
        
        # LiDAR sensor data
        self.scanner_pos = [0.0, 0.0, 0.0]        # Current scanner position [x, y, z] in meters
        self.measured_pos = None                    # Current measured target position [x, y, z] in meters
        self.signal_strength = 0.0                 # Current signal strength (0-100%)
        self.distance = 0.0                        # Current distance to target in meters
        
        # Motor control data
        self.pan_angle = 0.0                       # Current pan angle in degrees (-180 to +180)
        self.tilt_angle = 0.0                      # Current tilt angle in degrees (-90 to +90)
        
        # Position tracking
        self.position_history = []                  # List of previous target positions with timestamps
        self.last_position_update = None            # When the last position was recorded
        
        # Orbital calculations
        self.predicted_orbit_params = None         # Calculated orbital parameters
        self.last_orbit_calculation = None         # When orbital parameters were last calculated
        
        # System status and progress
        self.progress = 0.0                        # Current operation progress (0-100%)
        self.warning = None                        # Current warning message
        self.error = None                          # Current error message
        
        # Simulation control
        self.target_position = None                # Simulated target position for testing
        self.target_velocity = [0.0, 0.0, 0.0]    # Simulated target velocity
        
        # WebSocket connection management
        self.websocket = None                      # Current WebSocket connection
        self.client_connected = False              # Whether a client is connected
        
        print("Drone Tracker System initialized successfully")
    
    def set_state(self, new_state: str):
        """
        Change the system state
        
        This function safely changes the system's current state and logs the change.
        It's used throughout the system to track what the system is currently doing.
        
        Args:
            new_state (str): The new state to set
        """
        if new_state != self.current_state:
            print(f"System state changing from {self.current_state} to {new_state}")
            self.current_state = new_state
            
            # Reset progress when changing states
            self.progress = 0.0
            
            # Clear warnings and errors when changing states
            self.warning = None
            self.error = None
    
    def set_scanner_pos(self, x: float, y: float, z: float):
        """
        Update the scanner position
        
        This function updates where the LiDAR scanner beam is currently pointing.
        The scanner position is used to show the beam direction in the 3D visualization.
        
        Args:
            x (float): X coordinate in meters
            y (float): Y coordinate in meters
            z (float): Z coordinate in meters
        """
        self.scanner_pos = [x, y, z]
    
    def update_telemetry(self, pan: float, tilt: float, signal: float, dist: float):
        """
        Update the live telemetry data
        
        This function updates the real-time sensor readings that are displayed
        in the frontend interface.
        
        Args:
            pan (float): Pan angle in degrees
            tilt (float): Tilt angle in degrees
            signal (float): Signal strength percentage (0-100)
            dist (float): Distance to target in meters
        """
        self.pan_angle = pan
        self.tilt_angle = tilt
        self.signal_strength = signal
        self.distance = dist
    
    def add_position_to_history(self, position: List[float]):
        """
        Add a new position measurement to the history
        
        This function records where the target was detected, along with the current time.
        The position history is used to calculate orbital parameters and show movement patterns.
        
        Args:
            position (List[float]): 3D position [x, y, z] in meters
        """
        timestamp = int(time.time() * 1000)  # Current time in milliseconds
        
        # Add the new position to the history
        self.position_history.append({
            'position': position,
            'timestamp': timestamp
        })
        
        # Keep only the most recent positions to avoid memory issues
        max_history = 50  # Keep last 50 positions
        if len(self.position_history) > max_history:
            self.position_history = self.position_history[-max_history:]
        
        # Update the last position update time
        self.last_position_update = timestamp
        
        # Update the current measured position
        self.measured_pos = position
        
        print(f"Added position to history: {position}")
    
    def compute_orbital_parameters(self) -> Optional[Dict]:
        """
        Calculate orbital parameters from position history
        
        This function analyzes the recorded target positions to determine
        the orbital characteristics of the target. It uses simplified orbital mechanics
        to estimate parameters like orbit size, shape, and orientation.
        
        Returns:
            Optional[Dict]: Calculated orbital parameters or None if insufficient data
        """
        # Need at least MIN_POSITIONS_FOR_ORBIT positions to calculate orbit
        if len(self.position_history) < MIN_POSITIONS_FOR_ORBIT:
            return None
        
        try:
            # Extract positions from history
            positions = [entry['position'] for entry in self.position_history]
            
            # Calculate center of mass (approximate center of orbit)
            center_x = sum(pos[0] for pos in positions) / len(positions)
            center_y = sum(pos[1] for pos in positions) / len(positions)
            center_z = sum(pos[2] for pos in positions) / len(positions)
            
            # Calculate distances from center
            distances = []
            for pos in positions:
                dx = pos[0] - center_x
                dy = pos[1] - center_y
                dz = pos[2] - center_z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                distances.append(distance)
            
            # Calculate orbital parameters
            semi_major_axis = max(distances) / 1000.0  # Convert to kilometers
            eccentricity = (max(distances) - min(distances)) / (max(distances) + min(distances)) if max(distances) > 0 else 0
            
            # Calculate inclination (simplified)
            # This is a rough estimate based on the range of Y coordinates
            y_range = max(pos[1] for pos in positions) - min(pos[1] for pos in positions)
            inclination_rad = math.asin(y_range / (2 * max(distances))) if max(distances) > 0 else 0
            inclination_deg = math.degrees(inclination_rad)
            
            # Create the orbital parameters object
            orbit_params = {
                'elements': {
                    'semiMajorAxis': semi_major_axis,
                    'eccentricity': eccentricity,
                    'inclinationRad': inclination_rad,
                    'inclinationDeg': inclination_deg,
                    'raanDeg': 0.0,  # Simplified - not calculated
                    'argPeriapsisDeg': 0.0,  # Simplified - not calculated
                    'trueAnomalyDeg': 0.0  # Simplified - not calculated
                },
                'center': [center_x, center_y, center_z],
                'confidence': min(1.0, len(positions) / 20.0)  # Confidence based on number of measurements
            }
            
            print(f"Calculated orbital parameters: semi-major axis={semi_major_axis:.2f} km, eccentricity={eccentricity:.4f}")
            return orbit_params
            
        except Exception as e:
            print(f"Error calculating orbital parameters: {e}")
            return None
    
    def simulate_background_scan(self, elapsed_time: float):
        """
        Simulate the background scanning operation
        
        During background scan, the LiDAR sensor performs a 360-degree sweep
        to detect any potential targets in the environment.
        
        Args:
            elapsed_time (float): Time elapsed since scan started in seconds
        """
        # Calculate scan progress (0 to 1)
        scan_duration = 12.0  # 12 seconds for full 360-degree scan
        scan_progress = min(1.0, elapsed_time / scan_duration)
        
        # Update progress
        self.progress = scan_progress * 100.0
        
        # Calculate current scan angle (0 to 360 degrees)
        scan_angle = scan_progress * 360.0
        
        # Convert angle to radians
        angle_rad = math.radians(scan_angle)
        
        # Calculate scanner position (circular pattern at 8 meters radius)
        radius = 8.0
        x = radius * math.cos(angle_rad)
        y = 2.0  # Fixed height
        z = radius * math.sin(angle_rad)
        
        # Update scanner position
        self.set_scanner_pos(x, y, z)
        
        # Update telemetry
        self.update_telemetry(scan_angle, 0.0, 25.0, radius)
        
        # Check if scan is complete
        if scan_progress >= 1.0:
            print("Background scan completed - transitioning to target search")
            self.set_state(SystemState.SEARCHING_FOR_TARGET)
            self.simulation_start_time = time.time()  # Reset timer for search phase
            self.progress = 0.0
    
    def simulate_target_search(self, elapsed_time: float):
        """
        Simulate the target search operation
        
        During target search, the LiDAR sensor performs a systematic search pattern
        to locate a specific target in the environment.
        
        Args:
            elapsed_time (float): Time elapsed since search started in seconds
        """
        # Calculate search progress (0 to 1)
        search_duration = 15.0  # 15 seconds for complete search pattern
        search_progress = min(1.0, elapsed_time / search_duration)
        
        # Update progress
        self.progress = search_progress * 100.0
        
        # Create a zig-zag search pattern
        pattern_time = elapsed_time * 2.0  # Speed up the pattern
        x = 6.0 * math.sin(pattern_time * 2.0)  # Horizontal zig-zag
        y = 2.0 + 1.0 * math.sin(pattern_time * 3.0)  # Vertical variation
        z = 6.0 * math.cos(pattern_time * 1.5)  # Depth variation
        
        # Update scanner position
        self.set_scanner_pos(x, y, z)
        
        # Update telemetry
        pan_angle = math.degrees(math.atan2(x, z))
        tilt_angle = math.degrees(math.atan2(y, math.sqrt(x*x + z*z)))
        self.update_telemetry(pan_angle, tilt_angle, 35.0, math.sqrt(x*x + y*y + z*z))
        
        # Simulate target detection (increased chance as search progresses)
        detection_chance = 0.05 + (search_progress * 0.15)  # 5% to 20% chance
        if random.random() < detection_chance:
            print("Target detected during search! - transitioning to tracking")
            self.set_state(SystemState.TRACKING_TARGET)
            self.simulation_start_time = time.time()  # Reset timer for tracking phase
            self.progress = 0.0
            
            # Set target position near current scanner position
            self.target_position = [x + random.uniform(-1, 1), y + random.uniform(-1, 1), z + random.uniform(-1, 1)]
        
        # Check if search is complete without finding target
        elif search_progress >= 1.0:
            print("Target search completed without detection - returning to standby")
            self.set_state(SystemState.STANDBY)
            self.progress = 0.0
    
    def simulate_target_tracking(self, elapsed_time: float):
        """
        Simulate the target tracking operation
        
        During target tracking, the LiDAR sensor continuously monitors a detected target,
        updating its position and calculating orbital parameters.
        
        Args:
            elapsed_time (float): Time elapsed since tracking started in seconds
        """
        # Update progress (tracking is continuous)
        self.progress = min(100.0, elapsed_time * 10.0)  # Progress increases over time
        
        if not self.target_position:
            # Create a simulated target if none exists
            self.target_position = [5.0, 3.0, 4.0]
        
        # Simulate target movement (simple circular motion)
        target_time = elapsed_time * 0.5  # Slower movement
        radius = 5.0
        x = radius * math.cos(target_time)
        y = 3.0 + 1.0 * math.sin(target_time * 2.0)
        z = radius * math.sin(target_time)
        
        # Update target position
        self.target_position = [x, y, z]
        
        # Point scanner at target
        self.set_scanner_pos(x, y, z)
        
        # Calculate angles to target
        pan_angle = math.degrees(math.atan2(x, z))
        tilt_angle = math.degrees(math.atan2(y, math.sqrt(x*x + z*z)))
        distance = math.sqrt(x*x + y*y + z*z)
        
        # Update telemetry
        signal_strength = max(60.0, 100.0 - distance * 2.0)  # Signal decreases with distance
        self.update_telemetry(pan_angle, tilt_angle, signal_strength, distance)
        
        # Simulate occasional warnings and errors
        if elapsed_time > 30.0 and random.random() < 0.1:  # After 30 seconds, 10% chance
            if not self.warning:
                self.warning = "Target signal strength decreasing - consider recalibration"
                print(f"Warning: {self.warning}")
        
        if elapsed_time > 60.0 and random.random() < 0.05:  # After 60 seconds, 5% chance
            if not self.error:
                self.error = "Target tracking unstable - switching to search mode"
                print(f"Error: {self.error}")
                # Automatically switch back to search mode after error
                self.set_state(SystemState.SEARCHING_FOR_TARGET)
                self.simulation_start_time = time.time()
                self.progress = 0.0
                return
        
        # Add position to history periodically
        if not self.last_position_update or (time.time() * 1000 - self.last_position_update) > 1000:
            self.add_position_to_history(self.target_position)
        
        # Calculate orbital parameters periodically
        current_time = time.time() * 1000
        if (not self.last_orbit_calculation or 
            current_time - self.last_orbit_calculation > ORBIT_CALCULATION_INTERVAL_MS):
            
            self.predicted_orbit_params = self.compute_orbital_parameters()
            if self.predicted_orbit_params:
                self.last_orbit_calculation = current_time
                print(f"Orbital parameters updated - confidence: {self.predicted_orbit_params['confidence']:.2f}")
        
        # Simulate target loss after extended tracking (random event)
        if elapsed_time > 45.0 and random.random() < 0.02:  # After 45 seconds, 2% chance
            print("Target lost during tracking - returning to search mode")
            self.set_state(SystemState.SEARCHING_FOR_TARGET)
            self.simulation_start_time = time.time()
            self.progress = 0.0
            self.target_position = None
    
    def start_simulation(self):
        """
        Start the system simulation
        
        This function begins the main simulation loop and sets the system
        to the INITIALIZING state.
        """
        if self.is_running:
            print("Simulation is already running")
            return
        
        print("Starting system simulation...")
        self.is_running = True
        self.simulation_start_time = time.time()
        self.last_update_time = time.time()
        self.set_state(SystemState.INITIALIZING)
        
        # Initialize target position for simulation
        if not self.target_position:
            self.target_position = [5.0, 3.0, 4.0]
    
    def stop_simulation(self):
        """
        Stop the system simulation
        
        This function pauses the simulation and sets the system to the PAUSED state.
        The simulation can be resumed from where it left off.
        """
        if not self.is_running:
            print("Simulation is not running")
            return
        
        print("Stopping system simulation...")
        self.is_running = False
        self.set_state(SystemState.PAUSED)
    
    def reset_simulation(self):
        """
        Reset the system simulation
        
        This function completely resets the system to its initial state,
        clearing all data and returning to STANDBY.
        """
        print("Resetting system simulation...")
        
        # Stop simulation
        self.is_running = False
        
        # Reset all data
        self.scanner_pos = [0.0, 0.0, 0.0]
        self.measured_pos = None
        self.signal_strength = 0.0
        self.distance = 0.0
        self.pan_angle = 0.0
        self.tilt_angle = 0.0
        self.position_history = []
        self.predicted_orbit_params = None
        self.progress = 0.0
        self.warning = None
        self.error = None
        self.target_position = None
        
        # Reset timestamps
        self.simulation_start_time = None
        self.last_update_time = None
        self.last_position_update = None
        self.last_orbit_calculation = None
        
        # Return to standby
        self.set_state(SystemState.STANDBY)
        
        print("System reset complete")
    
    def run_simulation_step(self):
        """
        Run one step of the simulation
        
        This function updates the system state and generates new data
        based on the current state and elapsed time.
        """
        if not self.is_running:
            return
        
        current_time = time.time()
        elapsed_time = current_time - self.last_update_time
        
        # Update simulation time
        self.last_update_time = current_time
        
        # Handle different system states
        if self.current_state == SystemState.INITIALIZING:
            # Simulate initialization process
            init_duration = 3.0  # 3 seconds to initialize
            init_elapsed = current_time - self.simulation_start_time
            
            if init_elapsed >= init_duration:
                print("System initialization complete")
                self.set_state(SystemState.BACKGROUND_SCAN_IN_PROGRESS)
                self.simulation_start_time = current_time
            else:
                self.progress = (init_elapsed / init_duration) * 100.0
                
                # Simulate occasional warnings during initialization
                if init_elapsed > 1.5 and random.random() < 0.3 and not self.warning:
                    self.warning = "Calibrating LiDAR sensors - please wait"
                    print(f"Warning: {self.warning}")
        
        elif self.current_state == SystemState.BACKGROUND_SCAN_IN_PROGRESS:
            # Run background scan simulation
            scan_elapsed = current_time - self.simulation_start_time
            self.simulate_background_scan(scan_elapsed)
            
            # Simulate occasional warnings during scan
            if scan_elapsed > 6.0 and random.random() < 0.2 and not self.warning:
                self.warning = "Background scan in progress - monitoring environment"
                print(f"Warning: {self.warning}")
        
        elif self.current_state == SystemState.SEARCHING_FOR_TARGET:
            # Run target search simulation
            search_elapsed = current_time - self.simulation_start_time
            self.simulate_target_search(search_elapsed)
            
            # Simulate occasional warnings during search
            if search_elapsed > 8.0 and random.random() < 0.25 and not self.warning:
                self.warning = "Searching for targets - expanding search pattern"
                print(f"Warning: {self.warning}")
        
        elif self.current_state == SystemState.TRACKING_TARGET:
            # Run target tracking simulation
            tracking_elapsed = current_time - self.simulation_start_time
            self.simulate_target_tracking(tracking_elapsed)
    
    def get_system_state(self) -> Dict:
        """
        Get the current system state for transmission
        
        This function creates a data packet containing all the current
        system information that will be sent to the frontend.
        
        Returns:
            Dict: Complete system state data
        """
        return {
            'status': self.current_state,
            'progress': self.progress,
            'warning': self.warning,
            'error': self.error,
            'scanner_pos': self.scanner_pos,
            'measured_pos': self.measured_pos,
            'live_telemetry': {
                'pan_angle': self.pan_angle,
                'tilt_angle': self.tilt_angle,
                'signal_strength': self.signal_strength,
                'distance': self.distance
            },
            'position_history': self.position_history,
            'predicted_orbit_params': self.predicted_orbit_params,
            'timestamp': int(time.time() * 1000)
        }
    
    def get_status_summary(self) -> str:
        """
        Get a clean status summary for display
        
        Returns:
            str: Formatted status summary
        """
        status_parts = [f"Status: {self.current_state}"]
        
        if self.progress > 0:
            status_parts.append(f"Progress: {self.progress:.1f}%")
        
        if self.warning:
            status_parts.append(f"⚠️  {self.warning}")
        
        if self.error:
            status_parts.append(f"❌ {self.error}")
        
        if self.measured_pos:
            status_parts.append(f"Target: [{self.measured_pos[0]:.1f}, {self.measured_pos[1]:.1f}, {self.measured_pos[2]:.1f}]")
        
        if self.signal_strength > 0:
            status_parts.append(f"Signal: {self.signal_strength:.1f}%")
        
        return " | ".join(status_parts)
    
    def handle_command(self, command: str):
        """
        Handle commands from the frontend
        
        This function processes commands sent by the frontend to control
        the system's behavior.
        
        Args:
            command (str): The command to execute
        """
        print(f"Received command: {command}")
        
        if command == 'start':
            if self.current_state == SystemState.PAUSED:
                # Resume from paused state
                print("Resuming simulation from paused state")
                self.is_running = True
                self.simulation_start_time = time.time() - (self.last_update_time - self.simulation_start_time)
            else:
                # Start new simulation
                self.start_simulation()
        
        elif command == 'stop':
            self.stop_simulation()
        
        elif command == 'reset':
            self.reset_simulation()
        
        else:
            print(f"Unknown command: {command}")

    def clear_status_messages(self):
        """
        Clear status messages when they're no longer relevant
        
        This function automatically clears warnings and errors based on
        the current system state and timing.
        """
        current_time = time.time()
        
        # Only clear messages if we have a last update time
        if not self.last_update_time:
            return
            
        time_since_update = current_time - self.last_update_time
        
        # Clear warnings after a certain time (but not too frequently)
        if self.warning and time_since_update > 8.0:  # Increased from 5.0 to 8.0
            if "calibrating" in self.warning.lower() and self.current_state != SystemState.INITIALIZING:
                self.warning = None
            elif "background scan" in self.warning.lower() and self.current_state != SystemState.BACKGROUND_SCAN_IN_PROGRESS:
                self.warning = None
            elif "searching" in self.warning.lower() and self.current_state != SystemState.SEARCHING_FOR_TARGET:
                self.warning = None
            elif "monitoring" in self.warning.lower() and self.current_state != SystemState.BACKGROUND_SCAN_IN_PROGRESS:
                self.warning = None
        
        # Clear errors after a certain time
        if self.error and time_since_update > 15.0:  # Increased from 10.0 to 15.0
            self.error = None

async def websocket_handler(websocket, path, system: DroneTrackerSystem):
    """
    Handle WebSocket connections from clients
    
    This function processes incoming messages and sends system state updates
    to connected clients.
    
    Args:
        websocket: The WebSocket connection object
        path: The WebSocket path (not used)
        system: The drone tracker system instance
    """
    print(f"Client connected from {websocket.remote_address}")
    
    # Mark client as connected
    system.client_connected = True
    system.websocket = websocket
    
    try:
        # Send initial state
        initial_state = system.get_system_state()
        print(f"Initial state sent: {initial_state['status']}")
        await websocket.send(json.dumps(initial_state))
        print(f"Initial state data: {json.dumps(initial_state)[:100]}...")  # Show first 100 chars
        
        # Send a test message to verify connection
        test_msg = {"test": "connection_verified", "timestamp": int(time.time() * 1000)}
        await websocket.send(json.dumps(test_msg))
        print("Test message sent to verify connection")
        
        # Main message loop
        async for message in websocket:
            try:
                # Parse the incoming message
                data = json.loads(message)
                command = data.get('command')
                
                if command:
                    print(f"Command received: {command}")
                    # Handle the command
                    system.handle_command(command)
                    
                    # Send updated state
                    updated_state = system.get_system_state()
                    print(f"State updated: {updated_state['status']}")
                    await websocket.send(json.dumps(updated_state))
                    print(f"Updated state data: {json.dumps(updated_state)[:100]}...")  # Show first 100 chars
                else:
                    print(f"Message received without command: {data}")
                    
            except json.JSONDecodeError:
                print(f"Invalid JSON received: {message}")
            except Exception as e:
                print(f"Error processing message: {e}")
                
    except websockets.exceptions.ConnectionClosed:
        print(f"Client disconnected: {websocket.remote_address}")
    except Exception as e:
        print(f"WebSocket error: {e}")
    finally:
        # Clean up connection
        print(f"Connection closed for {websocket.remote_address}")
        system.client_connected = False
        system.websocket = None

async def simulation_loop(system: DroneTrackerSystem):
    """
    Main simulation loop
    
    This function runs the simulation at regular intervals,
    updating the system state and sending data to connected clients.
    
    Args:
        system: The drone tracker system instance
    """
    print("Starting simulation loop...")
    
    # Track last status to only log changes
    last_status = None
    last_progress = None
    data_send_count = 0
    
    while True:
        try:
            # Run simulation step
            system.run_simulation_step()
            
            # Clear status messages periodically (not every step)
            if data_send_count % 20 == 0:  # Only clear messages every 20 steps
                system.clear_status_messages()
            
            # Only log status changes
            current_status = system.current_state
            current_progress = system.progress
            
            if current_status != last_status or abs(current_progress - last_progress) > 5.0:
                print(system.get_status_summary())
                last_status = current_status
                last_progress = current_progress
            
            # Send updates to connected clients
            if system.client_connected and system.websocket:
                try:
                    state_data = system.get_system_state()
                    await system.websocket.send(json.dumps(state_data))
                    
                    # Only log data transmission every 20th time to reduce spam
                    data_send_count += 1
                    if data_send_count % 20 == 0:
                        print(f"Data sent to client ({data_send_count} updates) - Status: {state_data['status']}")
                        
                except Exception as e:
                    print(f"Error sending data to client: {e}")
                    system.client_connected = False
                    system.websocket = None
            else:
                # Only log disconnection status occasionally
                if data_send_count % 100 == 0:  # Reduced frequency
                    print(f"Waiting for client connection... (client_connected={system.client_connected})")
            
            # Wait for next update
            await asyncio.sleep(SIMULATION_STEP_TIME_MS / 1000.0)
            
        except Exception as e:
            print(f"Error in simulation loop: {e}")
            await asyncio.sleep(1.0)  # Wait before retrying

async def main():
    """
    Main application entry point
    
    This function sets up the WebSocket server and starts the simulation loop.
    """
    print("Enhanced Drone Tracker Digital Twin Backend")
    print("==========================================")
    print(f"WebSocket server starting on ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    
    # Create the system instance
    system = DroneTrackerSystem()
    
    # Create WebSocket server
    server = await websockets.serve(
        lambda ws, path: websocket_handler(ws, path, system),
        WEBSOCKET_HOST,
        WEBSOCKET_PORT
    )
    
    # System is now passed directly to the websocket_handler function
    
    print(f"Enhanced WebSocket server started on ws://{WEBSOCKET_HOST}:{WEBSOCKET_PORT}")
    print("Waiting for client to send 'start' command.")
    
    # Start simulation loop
    simulation_task = asyncio.create_task(simulation_loop(system))
    
    try:
        # Keep the server running
        await asyncio.Future()  # Run forever
    except KeyboardInterrupt:
        print("\nShutting down server...")
    finally:
        # Clean up
        simulation_task.cancel()
        server.close()
        await server.wait_closed()

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Enhanced Drone Tracker Digital Twin Backend")
    parser.add_argument('--simulate', action='store_true', help='Run in simulation mode')
    parser.add_argument('--host', default=WEBSOCKET_HOST, help='WebSocket host address')
    parser.add_argument('--port', type=int, default=WEBSOCKET_PORT, help='WebSocket port number')
    
    args = parser.parse_args()
    
    # Update configuration from command line arguments
    WEBSOCKET_HOST = args.host
    WEBSOCKET_PORT = args.port
    
    # Run the main application
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nServer stopped by user")
    except Exception as e:
        print(f"Server error: {e}")
