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
                coords_array = np.array([list(coord_tuple[0]) for coord_tuple in anomaly_averaged_coords])
                first_scan_positions = coords_array[:, :3]
                first_scan_timestamps = coords_array[:, 3:]
                kf.process_measurement_sequence(first_scan_positions, first_scan_timestamps)

                for dist, az_deg, el_deg in first_scan_positions:
                    initial_rad.append([dist, np.radians(az_deg), np.radians(el_deg)])
                
                anomaly_count = 0
                print(anomaly_averaged_coords)
                if should_change_state:
                    anomaly_averaged_coords = []
                    plot_data = first_scan_positions
                    current_state = states[2]

                print("Target detected! Starting EKF tracking...")
                # Clear LiDAR queue before starting
                while not lidar_data_queue.empty():
                    try:
                        lidar_data_queue.get_nowait()
                    except queue.Empty:
                        break

                max_tracking_cycles = 20
    
                while locked_in < max_tracking_cycles:
                    # Get Predicted positions 1 sec in future
                    current_time = first_scan_timestamps[-1] if len(first_scan_timestamps) > 0 else time.time()
                    future_time = current_time + 1
                    
                    predicted_positions = kf.predict_future_positions([future_time])
                    predicted_position = predicted_positions[0]  # Get first (and only) prediction
                    
                    # Convert to degrees and move motors there
                    pos_deg = [predicted_position[0], np.degrees(predicted_position[1]), np.degrees(predicted_position[2])]
                    current_azimuth, current_elevation, stepper_steps = move_to_polar_position(pi, pos_deg[1], pos_deg[2], stepper_steps)
                    
                    # Perform single detection scan
                    current_azimuth, current_elevation, stepper_steps, new_anomaly_coords, anomaly_count, should_change_state = perform_scanning_sequence(
                        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                        stepper_steps, anomaly_locations, [], 0, 1  # Reset anomaly tracking for single detection
                    )   
                    
                    # Process new measurement if found
                    if new_anomaly_coords and len(new_anomaly_coords) > 0:
                        # Extract the measurement data
                        coord_data = new_anomaly_coords[0][0]  # Get the tuple from the nested structure
                        current_measurement = [coord_data[0], coord_data[1], coord_data[2]]  # [distance, azimuth, elevation]
                        measurement_time = coord_data[3]  # timestamp
                        
                        # Update Kalman filter with the new measurement
                        # Note: update_with_delayed_measurement doesn't exist in your Kalman filter
                        # Use update_with_measurement_at_time instead
                        kf.update_with_measurement_at_time(
                            [current_measurement[0], np.radians(current_measurement[1]), np.radians(current_measurement[2])], 
                            measurement_time
                        )
                        
                        plot_data.append((current_measurement[0], current_measurement[1], current_measurement[2]))
                        print(f"Tracking cycle {locked_in + 1}: Updated with measurement at ({current_measurement[1]:.1f}°, {current_measurement[2]:.1f}°)")
                    else:
                        print(f"Tracking cycle {locked_in + 1}: No target detected at predicted position")
                    
                    locked_in += 1
                
                print("Tracking complete. Saving trajectory data...")
                save_calibration_data(plot_data)
                
                
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