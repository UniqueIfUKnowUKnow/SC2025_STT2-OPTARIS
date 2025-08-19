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
from enhanced_scanning import *


# --- WebSocket Bridge (React UI) ---
# Allow importing the RPi WS server from sat-track/scripts
import os, sys, asyncio
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WS_DIR = os.path.join(SCRIPT_DIR, '..', 'sat-track', 'scripts')
if WS_DIR not in sys.path:
    sys.path.append(WS_DIR)
try:
    from rpi_ws_server import start_server, bridge, geodetic_to_ecef, lidar_spherical_to_ecef_delta, control
except Exception as _ws_err:
    bridge = None
    start_server = None
    geodetic_to_ecef = None
    lidar_spherical_to_ecef_delta = None
    control = None
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
            # UI lifecycle control: wait for Start, handle Stop/Restart
            if control is not None:
                if control.is_stop_requested():
                    print("UI Stop requested. Returning to standby.")
                    control.acknowledge_stopped()
                    # Reset to initial state and loop back to wait for Start
                    current_state = states[0]
                    time.sleep(0.1)
                    continue
                if not control.is_running():
                    print("Waiting for UI Start...")
                    control.wait_for_start_blocking()
                    control.clear_start()
                    print("UI Start acknowledged. Running...")

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
                
                detections_required = 3
                # # #Sweeping for points
                # current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, calibration_done = perform_scanning_sequence(
                #     pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                #     stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count, detections_required
                # )
                
                current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, scanning_done = perform_point_to_point_sweep(
                    pi, lidar_data_queue, calibration_data, 5, 10,
                    20, 20, stepper_steps, anomaly_locations, 
                    anomaly_averaged_coords, anomaly_count, detections_required, 
                    num_steps=3, direction="forward")
                
                print(anomaly_count)
                print(anomaly_averaged_coords)
                if anomaly_count == 3:
                   detections_required = 1
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

                # Calculate unit vectors and fit plane
                unit_vectors = angles_to_unit(first_scan_pos_rad[:, 1], first_scan_pos_rad[:, 2])
                n_hat, _, _ = fit_plane_svd(unit_vectors.T)  # Note: fit_plane_svd expects Nx3 array
                
                # Calculate unit vector for first detected point and build plane basis
                first_unit_vector = unit_vectors[:, 0]  # First column is first unit vector
                cos_base, sin_base = build_plane_basis(n_hat, first_unit_vector)
                
                # Convert measurements into phase along the plane
                initial_phases = phase_from_unit(unit_vectors, cos_base, sin_base)
                
                # Unwrap phases to handle 2π transitions
                initial_phases_unwrapped = unwrap_phases(initial_phases)
                
                # Calculate least-square slope for initial angular rate
                t_mean = np.mean(first_scan_times)
                phase_mean = np.mean(initial_phases_unwrapped)

                numerator = np.sum((first_scan_times - t_mean) * (initial_phases_unwrapped - phase_mean))
                denominator = np.sum((first_scan_times - t_mean)**2)
                
                if abs(denominator) < 1e-10:
                    print("Warning: Cannot estimate angular rate - using default")
                    angular_speed = 0.1  # Default angular speed (rad/s)
                else:
                    angular_speed = numerator / denominator
                
                print(f"Initial phase-space tracking setup:")
                print(f"  Plane normal: n̂ = [{n_hat[0]:.3f}, {n_hat[1]:.3f}, {n_hat[2]:.3f}]")
                print(f"  Estimated angular speed: Ω = {angular_speed:.4f} rad/s ({np.degrees(angular_speed):.2f} deg/s)")
                print(f"  Initial phases: {np.degrees(initial_phases_unwrapped)}")
                
                # Initialize phase-space α-β filter
                # State: [phase, phase_rate] in radians and rad/s
                t_last = first_scan_times[-1]
                phase_filter = [initial_phases_unwrapped[-1], angular_speed]
                
                # Phase-space filter parameters (tuned for phase dynamics)
                ALPHA_PHASE = 0.4   # Position correction gain
                BETA_PHASE = 0.15   # Velocity correction gain
                
                tracking_iteration = 0
                max_tracking_iterations = 100
                
                # Store tracking history
                phase_history = list(initial_phases_unwrapped)
                time_history = list(first_scan_times)
                
                print(f"\n=== STARTING PHASE-SPACE TRACKING ===")
                print(f"Initial filter state: s = {np.degrees(phase_filter[0]):.1f}°, Ω = {np.degrees(phase_filter[1]):.3f} deg/s")
                
                while tracking_iteration < max_tracking_iterations:
                    tracking_iteration += 1
                    print(f"\n=== TRACKING ITERATION {tracking_iteration} ===")
                    
                    # Clear LiDAR queue before starting
                    while not lidar_data_queue.empty():
                        try:
                            lidar_data_queue.get_nowait()
                        except queue.Empty:
                            break

                    # PREDICTION STEP (entirely in phase space)
                    current_time = time.time()
                    dt = current_time - t_last
                    
                    if dt <= 0:
                        dt = DT  # Use default time step if timing is problematic
                    
                    # Predict next phase using constant angular velocity model
                    phase_pred = phase_filter[0] + phase_filter[1] * dt
                    phase_rate_pred = phase_filter[1]  # Constant velocity assumption
                    
                    print(f"Phase prediction:")
                    print(f"  Time step: dt = {dt:.3f}s")
                    print(f"  Predicted phase: s_pred = {np.degrees(phase_pred):.1f}° (unwrapped)")
                    print(f"  Predicted rate: Ω_pred = {np.degrees(phase_rate_pred):.3f} deg/s")
                    
                    # CONVERT PREDICTED PHASE TO AZ/EL FOR POINTING
                    # Reconstruct 3D unit vector from predicted phase
                    u_pred = cos_base * np.cos(phase_pred) + sin_base * np.sin(phase_pred)
                    
                    # Convert unit vector back to spherical coordinates
                    azi_pred, tilt_pred = unit_to_angles(u_pred)
                    
                    print(f"Pointing prediction:")
                    print(f"  Predicted azimuth: {np.degrees(azi_pred):.1f}°")
                    print(f"  Predicted elevation: {np.degrees(tilt_pred):.1f}°")
                    
                    # MEASUREMENT STEP - scan at predicted location
                    # anomaly_found, anomaly_measured, current_azimuth, current_elevation, stepper_steps = perform_targeted_scan(
                    #     pi, lidar_data_queue, calibration_data, np.degrees(azi_pred), np.degrees(tilt_pred),
                    #     stepper_steps)
                    anomaly_found, anomaly_measured, current_azimuth, current_elevation, stepper_steps, anomaly_count  = perform_point_to_point_sweep(
                    pi, lidar_data_queue, calibration_data, 5, 10,
                    20, 20, stepper_steps, anomaly_locations, 
                    anomaly_averaged_coords, anomaly_count, detections_required, 
                    num_steps=3, direction="forward")
                    
                    if not anomaly_found:
                        # Expand search if target not found at prediction
                        search_radius_deg = max(5.0, np.degrees(2.0 * np.sqrt(dt)))  # Adaptive search radius
                        print(f"Target not found at predicted location. Expanding search radius to ±{search_radius_deg:.1f}°")
                        
                        anomaly_found, anomaly_measured, current_azimuth, current_elevation, stepper_steps = perform_targeted_scan(
                            pi, lidar_data_queue, calibration_data, np.degrees(azi_pred), np.degrees(tilt_pred),
                            stepper_steps, search_radius_deg*2, search_radius_deg*2)

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
                        
                        # UPDATE STEP (convert measurement to phase space, then update filter)
                        
                        # Convert measured az/el to unit vector
                        azi_meas_rad = np.radians(anomaly_measured[1])
                        tilt_meas_rad = np.radians(anomaly_measured[2])
                        u_meas = angles_to_unit(azi_meas_rad, tilt_meas_rad).flatten()
                        
                        # Convert measured unit vector to phase
                        phase_meas = phase_from_unit(u_meas, cos_base, sin_base)
                        
                        # Handle phase wrapping - compute residual in wrapped space
                        phase_residual = wrap_to_pi(phase_meas - wrap_to_pi(phase_pred))
                        
                        print(f"Phase measurement and update:")
                        print(f"  Measured phase: s_meas = {np.degrees(phase_meas):.1f}° (wrapped)")
                        print(f"  Phase residual: Δs = {np.degrees(phase_residual):.2f}°")
                        
                        # α-β filter update in phase space
                        phase_updated = phase_pred + ALPHA_PHASE * phase_residual
                        phase_rate_updated = phase_rate_pred + (BETA_PHASE / dt) * phase_residual
                        
                        # Store updated filter state
                        phase_filter = [phase_updated, phase_rate_updated]
                        t_last = anomaly_measured[3]
                        
                        # Store for history (unwrap phase for logging)
                        if phase_history:
                            # Unwrap relative to last phase in history
                            last_phase = phase_history[-1]
                            phase_unwrapped = last_phase + wrap_to_pi(phase_updated - wrap_to_pi(last_phase))
                        else:
                            phase_unwrapped = phase_updated
                            
                        phase_history.append(phase_unwrapped)
                        time_history.append(t_last)
                        
                        print(f"Filter update:")
                        print(f"  Updated phase: s = {np.degrees(phase_updated):.1f}° (wrapped)")
                        print(f"  Updated rate: Ω = {np.degrees(phase_rate_updated):.3f} deg/s")
                        print(f"  Phase history length: {len(phase_history)}")
                        
                        # Store tracking data for later analysis
                        plot_data.append([anomaly_measured[0], anomaly_measured[1], anomaly_measured[2]])
                        
                        # Optional: Check for convergence or orbit completion
                        if len(phase_history) > 10:
                            phase_span = phase_history[-1] - phase_history[0]
                            if abs(phase_span) > 2*np.pi:
                                print(f"Completed one orbit! Phase span: {np.degrees(phase_span):.1f}°")
                                # Could break here if you want to stop after one orbit
                        
                    else:
                        print("TARGET LOST - Could not find target in expanded search area")
                        print("Continuing with prediction-only mode for one iteration...")
                        
                        # Update time but keep same filter state
                        t_last = current_time
                        
                        # Could implement more sophisticated lost-target handling here:
                        # - Increase search radius further
                        # - Reduce confidence in filter
                        # - Return to scanning mode
                        # For now, we'll just continue predicting
                        
                        if tracking_iteration > 5:  # Don't give up immediately
                            print("Target lost for too long, exiting tracking mode")
                            break
                
                print("Phase-space tracking complete. Saving trajectory data...")
                if plot_data:
                    save_calibration_data(plot_data)
                
                # Optional: Save phase history for analysis
                if len(phase_history) > 1:
                    phase_data = [[np.degrees(p), t, 0] for p, t in zip(phase_history, time_history)]
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    phase_filename = f"phase_tracking_{timestamp}.csv"
                    try:
                        with open(phase_filename, 'w', newline='') as csvfile:
                            writer = csv.writer(csvfile)
                            writer.writerow(['Phase_deg', 'Time_s', 'Placeholder'])
                            for row in phase_data:
                                writer.writerow(row)
                        print(f"✓ Phase tracking data saved to: {phase_filename}")
                    except Exception as e:
                        print(f"✗ Error saving phase data: {e}")
                
                
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