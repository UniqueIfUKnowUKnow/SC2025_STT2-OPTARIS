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
from zigzag import perform_targeted_scan


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
    
    anomaly_averaged_coords = []
    initial_rad = []
    first_scan_positions = []
    first_scan_timestamps = []
    plot_data = []
    locked_in = 0
    anomaly_count = 0
    anomaly_total = 0

    #Tracking constants for "DETECTED" state
    az_rad = []
    el_rad = []
    azi_filter = []
    tilt_filter = []
    ang_filter = []
    angvel_filter = []
    variance = []
    t_last = None
    anomaly_found = False

    
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
                # current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, calibration_done = perform_scanning_sequence(
                #     pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                #     stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, 3
                # )
                current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, calibration_done  = perform_targeted_scan(
                    pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                         stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, 3, 
                         10, 3)

                if calibration_done:
                    current_state = states[2]

            elif current_state == "DETECTED":
                
                anomaly_count = 0
                coords_array = np.array([list(coord_tuple[0]) for coord_tuple in anomaly_averaged_coords])
                first_scan_pos = coords_array[:, :3]
                first_scan_time = coords_array[:, 3:].flatten()
                
                # Convert degrees to radians for Kalman filter
                first_scan_pos_rad = []
                for i, pos in enumerate(first_scan_pos):
                    dist, az_deg, el_deg = pos[0], pos[1], pos[2]
                    # Convert to radians
                    az_rad.append(np.radians(az_deg))
                    el_rad.append(np.radians(el_deg))
                    first_scan_pos_rad.append([dist, az_rad, el_rad])
                
                #Calculate least-square slope
                t_mean = np.mean(first_scan_time)
                theta_mean = np.mean(az_rad)
                phi_mean = np.mean(el_rad)
                
                numerator = np.sum((first_scan_time - t_mean) * (az_rad - theta_mean))
                denominator = np.sum((first_scan_time - t_mean)**2)
                
                w_theta_ini = numerator / denominator
                
                numerator = np.sum((first_scan_time - t_mean) * (el_rad - phi_mean))
                w_phi_ini = numerator / denominator
                
                t_last = first_scan_time[-1]

                #default time step
                t_step = DT
                
                azi_filter = [theta_mean, t_step*w_theta_ini]
                tilt_filter = [phi_mean, t_step*w_phi_ini]

                while True:
                    
                        # Clear LiDAR queue before starting
                    while not lidar_data_queue.empty():
                        try:
                            lidar_data_queue.get_nowait()
                        except queue.Empty:
                            break

                    t_step = time.time() - t_last

                    azi_pred = azi_filter[0] + azi_filter[1] 
                    tilt_pred = tilt_filter[0] + tilt_filter[1]

                    #first scan at predicted area

                    anomaly_found, anomaly_measured, current_azimuth, current_elevation, stepper_steps = perform_local_search(
                        pi, lidar_data_queue, calibration_data, azi_pred, tilt_pred, stepper_steps)
                    
                    if not anomaly_found:
                        st_dev_azi = np.sqrt(sigma2_azi)
                        st_dev_tilt = np.sqrt(sigma2_tilt)
                        azi_half_width = np.max([W_MIN, KA * st_dev_azi])
                        tilt_half_width = np.max([W_MIN, KA * st_dev_tilt])

                        #TEMP AVERAGING
                        scan_width = np.degrees( (np.abs(azi_half_width) + np.abs(tilt_half_width)) / 2)
                        while not anomaly_found:
                            
                            anomaly_found, anomaly_measured, current_azimuth, current_elevation, stepper_steps = perform_local_search(
                            pi, lidar_data_queue, calibration_data, azi_pred, tilt_pred, stepper_steps, scan_width)

                    if anomaly_found:
                        #compute residuals (difference b/n measurement and prediction)
                        azi_residual = anomaly_measured[1] - azi_pred
                        tilt_residual = anomaly_measured[2] - tilt_pred

                        # Need to wrap azimuth around 2pi
                        azi_filter = [azi_pred + ALPHA_THETA * azi_residual, w_theta_ini + (BETA_THETA/t_step) * azi_residual]
                        tilt_filter = [tilt_pred + ALPHA_PHI * tilt_residual, w_phi_ini + (BETA_PHI/t_step) * tilt_residual]

                        sigma2_azi = LAMBDA * sigma2_azi + ( (1 - LAMBDA) * (azi_residual ** 2) ) + Q_AZI
                        sigma2_tilt = LAMBDA * sigma2_tilt + ( (1 - LAMBDA) * (tilt_residual ** 2) ) + Q_TILT
                        
                        t_last = anomaly_measured[3]





                    
                
                

                
                     

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