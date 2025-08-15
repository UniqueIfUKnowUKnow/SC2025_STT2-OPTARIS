# --- Standard Library Imports ---
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import pigpio            # For stable, hardware-based PWM for the servo
import time              # For creating delays
import serial            # For reading data from the LiDAR's serial port
import threading         # To run the LiDAR reader in the background
import queue             # For thread-safe data sharing between threads
import numpy as np
from lidar_reader import LidarReader
from constants import *
from stepper_setup import setup_stepper_gpio
from move_motors import *
from calibration import *
from tle_processing import parse_tle
from anomaly_check import get_interpolated_reference_distance
from scanning import perform_scanning_sequence
from datetime import datetime
from kalman_filter import DroneTrajectoryKalman
from tracking_functions import *


# --- Main Application ---
def main():
    """
    The main entry point of the program. It handles setup, state management,
    and the main control loop for the calibration and scanning process.
    """
    # --- Setup ---
    setup_stepper_gpio()
    pi = pigpio.pi()
    if not pi.connected:
        print("Could not connect to pigpio daemon. Is it running?")
        return
    GPIO.output(DIR_PIN, GPIO.HIGH)
    

    servo_angle = SERVO_SWEEP_START
    set_servo_angle(pi, servo_angle)


    lidar_data_queue = queue.Queue()
    lidar_thread = LidarReader(LIDAR_SERIAL_PORT, LIDAR_BAUD_RATE, lidar_data_queue)
    lidar_thread.start()

    kf = DroneTrajectoryKalman()

    tle_data = parse_tle([satellite_name, line1, line2])
    
    # --- State Machine and Variables ---
    states = ["CALIBRATING", "SCANNING", "DETECTED"]
    current_state = states[0]
    stepper_steps = 0
    current_azimuth = 0
    current_elevation = 0
    anomaly_detected = False
    anomaly_locations = []
    anomaly_count = 0
    anomaly_averaged_coords = []
    initial_rad = []
    first_scan_positions = []
    first_scan_timestamps = []
    plot_data = []
    locked_in = 0

    try:
        while True:
            if current_state == "CALIBRATING":
                print("Calibrating sensors...")

                # Mapping environment
                calibration_data = calibrate_environment(pi, lidar_data_queue)
                print(calibration_data)
                save_calibration_data(calibration_data)

                # Moving to right of ascending node
                current_azimuth, current_elevation, stepper_steps = move_to_polar_position(pi, tle_data["arg_perigee_deg"], 10 , stepper_steps)

                calibration_done = True
                if calibration_done:
                    current_state = states[1]

            # Corrected scanning section for main.py

            elif current_state == "SCANNING":
                print("Scanning area...")
                
                #Sweeping for points
                current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, should_change_state = perform_scanning_sequence(
                    pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                    stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, 5
                )
                
            elif current_state == "DETECTED":
                # Process initial detections and initialize Kalman filter
                coords_array = np.array([list(coord_tuple[0]) for coord_tuple in anomaly_averaged_coords])
                first_scan_positions = coords_array[:, :3]
                first_scan_timestamps = coords_array[:, 3:].flatten()  # Flatten to 1D array
                
                # Convert to radians for Kalman filter
                initial_rad = []
                for dist, az_deg, el_deg in first_scan_positions:
                    initial_rad.append([dist, np.radians(az_deg), np.radians(el_deg)])
                
                # Initialize Kalman filter
                kf.process_measurement_sequence(initial_rad, first_scan_timestamps)
                
                print("Target detected! Starting EKF tracking...")
                print(f"Initialized Kalman filter with {len(initial_rad)} measurements")
                
                # Clear LiDAR queue before starting tracking
                while not lidar_data_queue.empty():
                    try:
                        lidar_data_queue.get_nowait()
                    except queue.Empty:
                        break
                
                # Import the new tracking function
                from tracking_functions import perform_tracking_detection
                
                max_tracking_cycles = 20
                successful_detections = 0
                missed_detections = 0
                max_consecutive_misses = 3
                
                plot_data = first_scan_positions.tolist()  # Convert to list for appending
                
                locked_in = 0
                while locked_in < max_tracking_cycles:
                    cycle_start_time = time.time()
                    
                    print(f"\n=== TRACKING CYCLE {locked_in + 1}/{max_tracking_cycles} ===")
                    
                    # Get prediction for 1 second in the future
                    current_time = first_scan_timestamps[-1] if len(first_scan_timestamps) > 0 else time.time()
                    prediction_time = current_time + 1.0
                    
                    try:
                        predicted_positions = kf.predict_future_positions([prediction_time])
                        predicted_position = predicted_positions[0]  # [distance, azimuth_rad, elevation_rad]
                        
                        # Convert to degrees for motor control
                        pred_distance = predicted_position[0]
                        pred_azimuth_deg = np.degrees(predicted_position[1])
                        pred_elevation_deg = np.degrees(predicted_position[2])
                        
                        print(f"Predicted position: dist={pred_distance:.1f}m, az={pred_azimuth_deg:.1f}°, el={pred_elevation_deg:.1f}°")
                        
                        # Perform tracking detection at predicted location
                        new_measurement, current_azimuth, current_elevation, stepper_steps = perform_tracking_detection(
                            pi, lidar_data_queue, calibration_data, pred_azimuth_deg, pred_elevation_deg, stepper_steps
                        )
                        
                        if new_measurement is not None:
                            # Successful detection
                            successful_detections += 1
                            missed_detections = 0  # Reset consecutive miss counter
                            
                            # Extract measurement data: [distance, azimuth_deg, elevation_deg, timestamp]
                            meas_distance = new_measurement[0]
                            meas_azimuth_deg = new_measurement[1] 
                            meas_elevation_deg = new_measurement[2]
                            meas_timestamp = new_measurement[3]
                            
                            print(f"✓ Detection successful: {meas_distance:.1f}cm at ({meas_azimuth_deg:.1f}°, {meas_elevation_deg:.1f}°)")
                            
                            # Update Kalman filter with new measurement
                            meas_radians = [meas_distance, np.radians(meas_azimuth_deg), np.radians(meas_elevation_deg)]
                            kf.update_with_measurement_at_time(meas_radians, meas_timestamp)
                            
                            # Store for plotting
                            plot_data.append([meas_distance, meas_azimuth_deg, meas_elevation_deg])
                            
                            # Update timestamps for next prediction
                            first_scan_timestamps = np.append(first_scan_timestamps, meas_timestamp)
                            
                            print(f"Kalman filter updated. Total detections: {successful_detections}")
                            
                        else:
                            # Detection failed
                            missed_detections += 1
                            print(f"✗ Detection failed. Consecutive misses: {missed_detections}")
                            
                            if missed_detections >= max_consecutive_misses:
                                print(f"Too many consecutive misses ({missed_detections}). Target may be lost.")
                                break
                    
                    except Exception as e:
                        print(f"Error in tracking cycle: {e}")
                        missed_detections += 1
                        if missed_detections >= max_consecutive_misses:
                            break
                    
                    locked_in += 1
                    
                    # Show cycle timing
                    cycle_time = time.time() - cycle_start_time
                    print(f"Tracking cycle completed in {cycle_time:.2f} seconds")
                    
                    # Brief pause between cycles if needed
                    if cycle_time < 0.5:  # Ensure minimum cycle time
                        time.sleep(0.5 - cycle_time)
                
                # Tracking complete
                print(f"\n=== TRACKING SUMMARY ===")
                print(f"Completed {locked_in} tracking cycles")
                print(f"Successful detections: {successful_detections}")
                print(f"Total trajectory points: {len(plot_data)}")
                print("Saving trajectory data...")
                
                # Save the complete trajectory
                save_calibration_data(plot_data)
                
                print("Tracking complete. Returning to initial position...")
                # Return to starting position
                current_azimuth, current_elevation, stepper_steps = move_to_polar_position(pi, 0, 0, stepper_steps)
                
                # Exit or restart scanning
                break  # Exit the main loop, or set current_state = states[1] to restart scanning
                
                
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Cleaning up...")
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        reset_stepper_pos(stepper_steps)
        pi.stop()
        GPIO.cleanup()
        

if __name__ == '__main__':
    main()