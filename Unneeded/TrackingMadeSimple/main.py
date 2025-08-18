#!/usr/bin/env python3
# main.py
# Main system that uses every method/function in the algorithmic order/sequence
# Complete drone detection and tracking system for Raspberry Pi

# --- Standard Library Imports ---
import time
import sys
import signal
import threading
from datetime import datetime
import json
import os

# Import our custom modules
import firmware_StpInz as firmware
import background_scan
import find_first_position
import track_point_by_point
import data_processing

# --- System Configuration ---
# System states
SYSTEM_STATES = {
    "INITIALIZING": "Initializing hardware and sensors",
    "CALIBRATING": "Calibrating environment",
    "SEARCHING": "Searching for drone",
    "TRACKING": "Tracking drone movement",
    "PROCESSING": "Processing and outputting data",
    "ERROR": "System error state",
    "SHUTDOWN": "System shutdown"
}

# Configuration
DEFAULT_SEARCH_PATTERN = "spiral"  # Options: spiral, grid, concentric, zigzag
DEFAULT_TLE_DATA = {
    "name": "DRONE_TARGET",
    "line1": "1 25544U 98067A   24225.51782528  .00016717  00000-0  30199-3 0  9992",
    "line2": "2 25544  51.6426  95.0936 0007310  10.0000  60.6990 15.50209251454141"
}

# Tracking parameters
MAX_TRACKING_POINTS = 100
TRACKING_INTERVAL = 1.0  # seconds
MAX_SEARCH_TIME = 300    # seconds (5 minutes)

# Output settings
OUTPUT_FORMATS = ["json", "csv", "summary"]
SAVE_TRACKING_DATA = True

# --- System State Management ---
class DroneDetectionSystem:
    """Main system class for drone detection and tracking."""
    
    def __init__(self):
        self.current_state = "INITIALIZING"
        self.pi = None
        self.lidar_data_queue = None
        self.lidar_thread = None
        self.calibration_data = None
        self.tracked_positions = []
        self.current_position = None
        self.current_velocity = None
        self.system_running = False
        self.error_count = 0
        self.max_errors = 5
        
        # Data storage
        self.session_data = {
            "start_time": None,
            "end_time": None,
            "total_positions": 0,
            "search_time": 0,
            "tracking_time": 0,
            "errors": []
        }
    
    def change_state(self, new_state):
        """Change system state and log the transition."""
        old_state = self.current_state
        self.current_state = new_state
        print(f"\n=== STATE CHANGE: {old_state} -> {new_state} ===")
        print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"Description: {SYSTEM_STATES.get(new_state, 'Unknown state')}")
    
    def log_error(self, error_msg, error_type="SYSTEM"):
        """Log system errors."""
        error_entry = {
            "timestamp": datetime.now().isoformat(),
            "type": error_type,
            "message": error_msg,
            "state": self.current_state
        }
        self.session_data["errors"].append(error_entry)
        self.error_count += 1
        print(f"ERROR [{error_type}]: {error_msg}")
        
        if self.error_count >= self.max_errors:
            self.change_state("ERROR")
            return False
        return True
    
    def initialize_system(self):
        """Initialize the complete system."""
        print("=== DRONE DETECTION SYSTEM INITIALIZATION ===")
        print(f"Start time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        try:
            # Initialize firmware components
            self.pi, self.lidar_data_queue, self.lidar_thread = firmware.initialize_firmware()
            
            if not self.pi or not self.lidar_data_queue:
                raise Exception("Failed to initialize firmware components")
            
            print("✓ Firmware initialization complete")
            
            # Wait for LiDAR to stabilize
            print("Waiting for LiDAR to stabilize...")
            time.sleep(2)
            
            self.change_state("CALIBRATING")
            return True
            
        except Exception as e:
            self.log_error(f"Initialization failed: {e}", "INITIALIZATION")
            return False
    
    def calibrate_environment(self):
        """Calibrate the environment for anomaly detection."""
        print("=== ENVIRONMENT CALIBRATION ===")
        
        try:
            # Perform environment calibration
            self.calibration_data = background_scan.calibrate_environment(
                self.pi, self.lidar_data_queue
            )
            
            if not self.calibration_data:
                raise Exception("Calibration failed - no data collected")
            
            # Save calibration data
            background_scan.save_calibration_data(self.calibration_data)
            
            print(f"✓ Calibration complete - {len(self.calibration_data)} data points")
            self.change_state("SEARCHING")
            return True
            
        except Exception as e:
            self.log_error(f"Calibration failed: {e}", "CALIBRATION")
            return False
    
    def search_for_drone(self):
        """Search for the first drone position."""
        print("=== DRONE SEARCH PHASE ===")
        
        try:
            search_start_time = time.time()
            
            # Search for drone using specified pattern
            found, position, distance, search_time = find_first_position.find_first_drone_position(
                self.pi, self.lidar_data_queue, self.calibration_data, DEFAULT_SEARCH_PATTERN
            )
            
            self.session_data["search_time"] = search_time
            
            if found:
                print(f"✓ DRONE FOUND at position: {position}")
                print(f"Distance: {distance:.1f}cm")
                print(f"Search time: {search_time:.2f} seconds")
                
                # Create DronePosition object
                self.current_position = track_point_by_point.DronePosition(
                    position[0], position[1], distance, time.time()
                )
                
                self.change_state("TRACKING")
                return True
            else:
                print("✗ No drone found during search")
                print(f"Search time: {search_time:.2f} seconds")
                
                # Check if we should continue searching or give up
                if search_time < MAX_SEARCH_TIME:
                    print("Continuing search with different pattern...")
                    return self.search_for_drone()  # Recursive call with different pattern
                else:
                    print("Maximum search time reached")
                    self.change_state("ERROR")
                    return False
                    
        except Exception as e:
            self.log_error(f"Search failed: {e}", "SEARCH")
            return False
    
    def track_drone(self):
        """Track the drone point by point."""
        print("=== DRONE TRACKING PHASE ===")
        
        try:
            tracking_start_time = time.time()
            
            # Start point-by-point tracking
            self.tracked_positions = track_point_by_point.track_drone_point_by_point(
                self.pi, self.lidar_data_queue, self.calibration_data,
                self.current_position, MAX_TRACKING_POINTS, TRACKING_INTERVAL
            )
            
            tracking_time = time.time() - tracking_start_time
            self.session_data["tracking_time"] = tracking_time
            self.session_data["total_positions"] = len(self.tracked_positions)
            
            print(f"✓ Tracking complete - {len(self.tracked_positions)} positions tracked")
            print(f"Tracking time: {tracking_time:.2f} seconds")
            
            # Calculate final velocity from last two positions
            if len(self.tracked_positions) >= 2:
                self.current_velocity = track_point_by_point.calculate_velocity(
                    self.tracked_positions[-2], self.tracked_positions[-1]
                )
                self.current_position = self.tracked_positions[-1]
            
            self.change_state("PROCESSING")
            return True
            
        except Exception as e:
            self.log_error(f"Tracking failed: {e}", "TRACKING")
            return False
    
    def process_and_output_data(self):
        """Process and output all collected data."""
        print("=== DATA PROCESSING AND OUTPUT ===")
        
        try:
            # Process data for each tracked position
            processed_data_list = []
            
            for i, position in enumerate(self.tracked_positions):
                # Calculate velocity for this position
                velocity = None
                if i > 0:
                    velocity = track_point_by_point.calculate_velocity(
                        self.tracked_positions[i-1], position
                    )
                
                # Get recent positions for confidence calculation
                recent_positions = self.tracked_positions[max(0, i-5):i]
                
                # Process and output data
                processed_data = data_processing.process_and_output_drone_data(
                    position, velocity, DEFAULT_TLE_DATA, recent_positions, OUTPUT_FORMATS
                )
                processed_data_list.append(processed_data)
                
                print(f"Processed position {i+1}/{len(self.tracked_positions)}")
            
            # Save complete tracking session data
            if SAVE_TRACKING_DATA:
                self.save_session_data(processed_data_list)
            
            print(f"✓ Data processing complete - {len(processed_data_list)} positions processed")
            self.change_state("SHUTDOWN")
            return True
            
        except Exception as e:
            self.log_error(f"Data processing failed: {e}", "PROCESSING")
            return False
    
    def save_session_data(self, processed_data_list):
        """Save complete session data."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_filename = f"drone_session_{timestamp}.json"
        
        session_data = {
            "session_info": {
                "start_time": self.session_data["start_time"].isoformat() if self.session_data["start_time"] else None,
                "end_time": datetime.now().isoformat(),
                "total_positions": self.session_data["total_positions"],
                "search_time": self.session_data["search_time"],
                "tracking_time": self.session_data["tracking_time"],
                "error_count": self.error_count
            },
            "errors": self.session_data["errors"],
            "tracked_positions": [
                {
                    "azimuth": pos.azimuth,
                    "elevation": pos.elevation,
                    "distance": pos.distance,
                    "timestamp": pos.timestamp
                } for pos in self.tracked_positions
            ],
            "processed_data": [
                {
                    "timestamp": data.timestamp.isoformat(),
                    "position": {
                        "azimuth": data.position.azimuth,
                        "elevation": data.position.elevation,
                        "distance": data.position.distance
                    },
                    "velocity": None if data.velocity is None else {
                        "azimuth_velocity": data.velocity.azimuth_velocity,
                        "elevation_velocity": data.velocity.elevation_velocity,
                        "distance_velocity": data.velocity.distance_velocity
                    },
                    "confidence": data.confidence,
                    "status": data.status
                } for data in processed_data_list
            ]
        }
        
        try:
            with open(session_filename, 'w') as f:
                json.dump(session_data, f, indent=2)
            print(f"✓ Session data saved to: {session_filename}")
        except Exception as e:
            print(f"✗ Error saving session data: {e}")
    
    def cleanup_system(self):
        """Clean up system resources."""
        print("=== SYSTEM CLEANUP ===")
        
        try:
            # Cleanup firmware
            if self.pi:
                firmware.cleanup_firmware(self.pi)
            
            # Update session data
            self.session_data["end_time"] = datetime.now()
            
            print("✓ System cleanup complete")
            
        except Exception as e:
            print(f"✗ Error during cleanup: {e}")
    
    def run_system(self):
        """Main system execution loop."""
        print("=== STARTING DRONE DETECTION SYSTEM ===")
        
        self.system_running = True
        self.session_data["start_time"] = datetime.now()
        
        try:
            # System execution flow
            if not self.initialize_system():
                return False
            
            if not self.calibrate_environment():
                return False
            
            if not self.search_for_drone():
                return False
            
            if not self.track_drone():
                return False
            
            if not self.process_and_output_data():
                return False
            
            return True
            
        except KeyboardInterrupt:
            print("\nSystem interrupted by user")
            return False
        except Exception as e:
            self.log_error(f"System execution failed: {e}", "SYSTEM")
            return False
        finally:
            self.cleanup_system()
            self.system_running = False

# --- Signal Handlers ---
def signal_handler(signum, frame):
    """Handle system signals for graceful shutdown."""
    print(f"\nReceived signal {signum}. Shutting down gracefully...")
    if system_instance and system_instance.system_running:
        system_instance.system_running = False

# --- Main Execution ---
if __name__ == "__main__":
    print("DRONE DETECTION AND TRACKING SYSTEM")
    print("=" * 50)
    print("System components:")
    print("  - Firmware initialization (GPIO, motors, LiDAR)")
    print("  - Environment calibration")
    print("  - Drone search (multiple patterns)")
    print("  - Point-by-point tracking")
    print("  - Data processing and output")
    print("=" * 50)
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run system
    system_instance = DroneDetectionSystem()
    
    try:
        success = system_instance.run_system()
        
        if success:
            print("\n🎉 SYSTEM EXECUTION COMPLETED SUCCESSFULLY!")
            print("All phases completed:")
            print("  ✓ Initialization")
            print("  ✓ Calibration")
            print("  ✓ Drone search")
            print("  ✓ Tracking")
            print("  ✓ Data processing")
        else:
            print("\n❌ SYSTEM EXECUTION FAILED!")
            print(f"Failed at state: {system_instance.current_state}")
            print(f"Errors encountered: {system_instance.error_count}")
        
        # Print final summary
        print(f"\n=== FINAL SUMMARY ===")
        print(f"Total positions tracked: {system_instance.session_data['total_positions']}")
        print(f"Search time: {system_instance.session_data['search_time']:.2f} seconds")
        print(f"Tracking time: {system_instance.session_data['tracking_time']:.2f} seconds")
        print(f"Total errors: {system_instance.error_count}")
        
    except Exception as e:
        print(f"\n💥 CRITICAL SYSTEM ERROR: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n=== SYSTEM SHUTDOWN COMPLETE ===")
        print("Thank you for using the Drone Detection System!")
