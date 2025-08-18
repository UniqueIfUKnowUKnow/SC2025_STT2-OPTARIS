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
from tracking_functions import *
from zigzag import perform_targeted_scan
from coordinate_transfer import *


# --- WebSocket Bridge (React UI) ---
# Allow importing the RPi WS server from sat-track/scripts
import os, sys, asyncio
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WS_DIR = os.path.join(SCRIPT_DIR, '..', 'sat-track', 'scripts')
if WS_DIR not in sys.path:
    sys.path.append(WS_DIR)
try:
    from rpi_ws_server import start_server, bridge, geodetic_to_ecef, lidar_spherical_to_ecef_delta
except Exception as _ws_err:
    bridge = None
    start_server = None
    geodetic_to_ecef = None
    lidar_spherical_to_ecef_delta = None
    print("[WS] WebSocket bridge not available:", _ws_err)


# Ground station location (Sofia Tech Park), used for ECEF trajectory
GS_LAT = 42.6501
GS_LON = 23.3795
GS_ALT_KM = 0.6


def _start_ws_server_in_thread():
    """Start the RPi WebSocket server in a background thread."""
    if start_server is None:
        return

    def _run():
        try:
            asyncio.run(start_server(run_forever=True))
        except Exception as e:
            print("[WS] Server thread error:", e)

    threading.Thread(target=_run, daemon=True).start()


def _safe_push(payload):
    """Push JSON to UI if bridge is available."""
    try:
        if bridge is not None:
            bridge.push_sync(payload)
    except Exception as e:
        print("[WS] push error:", e)


# --- Main Application ---
def main():
    """
    The main entry point of the program. It handles setup, state management,
    and the main control loop for the calibration and scanning process.
    """
    # --- Setup ---
    _start_ws_server_in_thread()
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
    # Push initial TLE to UI for initial orbit
    _safe_push({
        "initial_tle1": line1,
        "initial_tle2": line2
    })
    
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
    first_scan_pos_rad = []
    az_rad = []
    el_rad = []
    azi_filter = []
    tilt_filter = []
    ang_filter = []
    angvel_filter = []
    variance = []
    t_last = None
    anomaly_found = False
    sigma2_azi = np.radians(3)**2
    sigma2_tilt = np.radians(3)**2
    cos_base = None
    sin_base = None
    phases = []

    # Global trajectory buffer for UI (ECEF km)
    global_traj = []

    
    try:
        while True:
            if current_state == "CALIBRATING":
                print("Calibrating sensors...")
                _safe_push({
                    "status": "CALIBRATING",
                    "progress": 0,
                    "live_telemetry": {"statusMessage": "Calibrating sensors..."}
                })

                # Mapping environment
                calibration_data = calibrate_environment(pi, lidar_data_queue)
                print(calibration_data)
                save_calibration_data(calibration_data)
                _safe_push({
                    "status": "CALIBRATING",
                    "progress": 100,
                    "live_telemetry": {"statusMessage": "Calibration complete"}
                })

                # Moving to right of ascending node
                current_azimuth, current_elevation, stepper_steps = move_to_polar_position(pi, tle_data["arg_perigee_deg"], 10 , stepper_steps)

                calibration_done = True
                if calibration_done:
                    current_state = states[1]

            # Corrected scanning section for main.py

            elif current_state == "SCANNING":
                print("Scanning area...")
                _safe_push({
                    "status": "SCANNING",
                    "live_telemetry": {
                        "pan_angle": float(current_azimuth),
                        "tilt_angle": float(current_elevation)
                    }
                })
                
                
                #Sweeping for points
                current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, calibration_done = perform_scanning_sequence(
                    pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                    stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, 3
                )

                if calibration_done:
                    current_state = states[2]

            elif current_state == "DETECTED":
                
                anomaly_count = 0
                coords_array = np.array([list(coord_tuple[0]) for coord_tuple in anomaly_averaged_coords])
                first_scan_pos = coords_array[:, :3]
                first_scan_times = coords_array[:, 3:].flatten()
                
                
                # RESET DIRECTION PIN TO KNOWN STATE BEFORE TRACKING
                print("Resetting direction pin to known state for tracking...")
                GPIO.output(DIR_PIN, GPIO.HIGH)
                time.sleep(0.002)  # Allow direction to settle
                
                # Convert degrees to radians
                first_scan_pos_rad = degrees_to_radians(first_scan_pos)

                # Calculate unit vector of plane of best fit
                n_hat = fit_plane_svd(first_scan_pos_rad)

                # Calculate unit vector for first detected point
                unit_vector = angles_to_unit(first_scan_pos[0, 1], first_scan_pos[0, 2])
                unit_vectors = angles_to_unit(first_scan_pos[:, 1], first_scan_pos[:, 2])
                
                #C and S bases for phases along the plane
                cos_base, sin_base =  build_plane_basis(n_hat, unit_vector)

                #Convert measurements into phase along inclined plane
                initial_phases = phase_from_unit(unit_vectors, cos_base, sin_base)

                #Calculate least-square slope
                t_mean = np.mean(first_scan_times)
                phase_mean = np.mean(initial_phases)

                numerator = np.sum((first_scan_times - t_mean) * (initial_phases - phase_mean))
                denominator = np.sum((first_scan_times - t_mean)**2)
                
                angular_speed = numerator / denominator
                
                t_last = first_scan_times[-1]

                #default time step
                t_step = DT
                
                phase_filter = [initial_phases[-1], angular_speed]

                tracking_iteration = 0
                max_tracking_iterations = 100  # Safety limit
                
                phases.append(initial_phases)
                while tracking_iteration < max_tracking_iterations:
                    tracking_iteration += 1
                    print(f"\n=== TRACKING ITERATION {tracking_iteration} ===")
                    
                    # Clear LiDAR queue before starting
                    while not lidar_data_queue.empty():
                        try:
                            lidar_data_queue.get_nowait()
                        except queue.Empty:
                            break

                    t_step = time.time() - t_last


                    phase_pred = phase_filter[-1] + phase_filter[1] * t_step
                    phases.append(phase_pred)
                    u_pred = cos_base*np.cos(phase_pred) + sin_base*np.sin(phase_pred) 
                    azi_pred, tilt_pred = unit_to_angles(u_pred)

                    #first scan at predicted area
                    anomaly_found, anomaly_measured, current_azimuth, current_elevation, stepper_steps = perform_targeted_scan(
                                pi, lidar_data_queue, calibration_data, np.degrees(azi_pred), np.degrees(tilt_pred),
                                stepper_steps)
                    
                    if not anomaly_found:
                        st_dev_azi = np.sqrt(sigma2_azi)
                        st_dev_tilt = np.sqrt(sigma2_tilt)
                        azi_half_width = np.max([W_MIN, KA * st_dev_azi])
                        tilt_half_width = np.max([W_MIN, KA * st_dev_tilt])

                        print(f"Target not found at predicted location. Expanding search...")
                        print(f"Search area: Az±{np.degrees(azi_half_width):.1f}°, El±{np.degrees(tilt_half_width):.1f}°")
                        
                        # Expanded search
                        anomaly_found, anomaly_measured, current_azimuth, current_elevation, stepper_steps = perform_targeted_scan(
                            pi, lidar_data_queue, calibration_data, np.degrees(azi_pred), np.degrees(tilt_pred),
                            stepper_steps, np.degrees(azi_half_width*2), np.degrees(tilt_half_width*2))

                    if anomaly_found:
                        print(f"TARGET FOUND at Az={anomaly_measured[1]:.1f}°, El={anomaly_measured[2]:.1f}°")
                        # Push live telemetry and trajectory point to UI
                        try:
                            distance_m = float(anomaly_measured[0])
                            az_deg = float(anomaly_measured[1])
                            el_deg = float(anomaly_measured[2])
                            raw_ts = anomaly_measured[3]
                            ts_ms = int(raw_ts * 1000) if raw_ts < 1e12 else int(raw_ts)

                            if geodetic_to_ecef and lidar_spherical_to_ecef_delta:
                                obs_x, obs_y, obs_z = geodetic_to_ecef(GS_LAT, GS_LON, GS_ALT_KM)
                                dx, dy, dz = lidar_spherical_to_ecef_delta(distance_m, az_deg, el_deg, GS_LAT, GS_LON)
                                x_km = obs_x + dx
                                y_km = obs_y + dy
                                z_km = obs_z + dz
                                global_traj.append({"x": x_km, "y": y_km, "z": z_km, "timestamp": ts_ms})
                                if len(global_traj) > 1000:
                                    global_traj = global_traj[-1000:]

                            _safe_push({
                                "status": "TRACKING_TARGET",
                                "live_telemetry": {
                                    "pan_angle": az_deg,
                                    "tilt_angle": el_deg,
                                    "distance": distance_m
                                },
                                "trajectory_points": global_traj[-200:]
                            })
                        except Exception as e:
                            print("[WS] DETECTED publish error:", e)
                        
                        #compute residuals (difference between measurement and prediction)
                        azi_residual = np.radians(anomaly_measured[1]) - azi_pred
                        tilt_residual = np.radians(anomaly_measured[2]) - tilt_pred

#WRAP TO PI HERE

                        print(f"Residuals: Az={np.degrees(azi_residual):.2f}°, El={np.degrees(tilt_residual):.2f}°")
                        
                        # Update filter estimates
                        azi_filter = [azi_pred + ALPHA_THETA * azi_residual, azi_filter[1] + (BETA_THETA/t_step) * azi_residual]
                        tilt_filter = [tilt_pred + ALPHA_PHI * tilt_residual, tilt_filter[1] + (BETA_PHI/t_step) * tilt_residual]

#WRAP TO PI HERE
                        
                        # Update variance estimates
                        sigma2_azi = LAMBDA * sigma2_azi + ( (1 - LAMBDA) * (azi_residual ** 2) ) + Q_AZI
                        sigma2_tilt = LAMBDA * sigma2_tilt + ( (1 - LAMBDA) * (tilt_residual ** 2) ) + Q_TILT
                        
                        t_last = anomaly_measured[3]
                        
                        print(f"Updated estimates: Az={np.degrees(azi_filter[0]):.1f}°, El={np.degrees(tilt_filter[0]):.1f}°")
                        print(f"Updated velocities: Az_vel={np.degrees(azi_filter[1]):.2f}°/s, El_vel={np.degrees(tilt_filter[1]):.2f}°/s")
                        
                        # Store tracking data for later analysis
                        plot_data.append([anomaly_measured[0], anomaly_measured[1], anomaly_measured[2]])
                        
                    else:
                        print("TARGET LOST - Could not find target in expanded search area")
                        print("Continuing with prediction-only mode...")
                        # You might want to expand search further or exit tracking
                        break
                
                print("Tracking complete. Saving trajectory data...")
                if plot_data:
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