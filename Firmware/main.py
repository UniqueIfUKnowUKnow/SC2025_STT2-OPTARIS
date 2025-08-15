# --- Standard Library Imports ---
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import pigpio            # For stable, hardware-based PWM for the servo
import time              # For creating delays
import serial            # For reading data from the LiDAR's serial port
import threading         # To run the LiDAR reader in the background
import queue             # For thread-safe data sharing between threads
from lidar_reader import LidarReader
from constants import *
from stepper_setup import setup_stepper_gpio
from move_motors import *
from calibration import *
from tle_processing import parse_tle
from anomaly_check import get_interpolated_reference_distance
from scanning import perform_scanning_sequence
from ekf_tracker import create_drone_tracker, update_tracker_with_anomaly_data
from datetime import datetime
import math
from ekf_tracker import DroneEK


def enhanced_detected_state(pi, lidar_data_queue, calibration_data, anomaly_averaged_coords, stepper_steps):
    """
    Enhanced DETECTED state that implements drone tracking and predictive targeting.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: LiDAR data queue
        calibration_data: Calibration reference data
        anomaly_averaged_coords: List of detected anomaly coordinates with timestamps
        stepper_steps: Current stepper position
        
    Returns:
        bool: True if tracking should continue, False if done
    """
    print("=== DRONE DETECTION AND TRACKING MODE ===")
    
    # Initialize the drone tracker
    if len(anomaly_averaged_coords) > 0:
        # Use first detection as initial estimate
        first_detection = anomaly_averaged_coords[0][0]  # [distance, azimuth, elevation, timestamp]
        tracker = create_drone_tracker(initial_detection=first_detection[:3])
        print(f"Tracker initialized with first detection: {first_detection[:3]}")
    else:
        tracker = create_drone_tracker()
        print("Tracker initialized without initial detection")
    
    # Update tracker with all existing anomaly data
    tracker = update_tracker_with_anomaly_data(tracker, anomaly_averaged_coords)
    
    print(f"Tracker updated with {tracker.get_measurement_count()} measurements")
    print(f"Tracking confidence: {tracker.get_tracking_confidence():.2f}")
    print(f"Current drone speed: {tracker.get_current_velocity_magnitude():.2f} m/s")
    
    # Main tracking loop
    tracking_duration = 30  # Track for 30 seconds
    prediction_time = 2.0   # Predict 2 seconds ahead
    
    start_time = datetime.now()
    last_move_time = datetime.now()
    move_interval = 0.5  # Move motors every 0.5 seconds
    
    while (datetime.now() - start_time).total_seconds() < tracking_duration:
        current_time = datetime.now()
        
        # Get new LiDAR measurements and check for continued detections
        try:
            # In a real implementation, you'd need to distinguish between
            # background objects and the moving drone. This is simplified.
            distance = lidar_data_queue.get_nowait()
            
            # Get current motor position for measurement
            current_azimuth = (stepper_steps / STEPS_PER_REVOLUTION) * 360
            current_elevation = SERVO_SWEEP_START  # Assuming servo position is known
            
            # Simple anomaly detection (you might want to use your existing anomaly detection)
            reference = get_interpolated_reference_distance(
                current_azimuth, current_elevation, calibration_data
            )
            
            if distance < reference * ANOMALY_FACTOR:
                # New detection - update tracker
                measurement = [distance, current_azimuth, current_elevation]
                tracker.update(measurement, current_time)
                
                print(f"New detection: {distance:.1f}cm at ({current_azimuth:.1f}°, {current_elevation:.1f}°)")
                print(f"Updated tracking confidence: {tracker.get_tracking_confidence():.2f}")
                
        except queue.Empty:
            pass
        
        # Move motors to predicted position periodically
        if (current_time - last_move_time).total_seconds() >= move_interval:
            if tracker.get_measurement_count() >= 3:  # Need enough data for prediction
                try:
                    # Use validated prediction to prevent wrong-direction tracking
                    future_state = tracker.predict_with_validation(prediction_time)
                    x, y, z = future_state[0], future_state[1], future_state[2]
                    
                    # Convert to spherical coordinates for motor control
                    azimuth_deg, elevation_deg, distance_m = xyz_to_polar(x, y, z)
                    future_distance = distance_m * 100.0  # Convert to cm
                    
                    print(f"Predicting drone at t+{prediction_time}s: "
                          f"{future_distance:.0f}cm, {azimuth_deg:.1f}°, {elevation_deg:.1f}°")
                    
                    # Validate the prediction makes sense (not too far from current position)
                    current_pos = tracker.x[:3]
                    current_distance = math.sqrt(current_pos[0]**2 + current_pos[1]**2 + current_pos[2]**2) * 100
                    
                    distance_change = abs(future_distance - current_distance)
                    max_reasonable_change = tracker.get_current_velocity_magnitude() * prediction_time * 100 * 1.5  # Allow 50% margin
                    
                    if distance_change > max_reasonable_change:
                        print(f"Warning: Predicted distance change ({distance_change:.0f}cm) seems too large!")
                        print("Using current position instead of prediction")
                        # Use current position instead
                        current_azimuth_deg, current_elevation_deg, current_dist_m = xyz_to_polar(
                            current_pos[0], current_pos[1], current_pos[2]
                        )
                        azimuth_deg, elevation_deg = current_azimuth_deg, current_elevation_deg
                    
                    # Move motors to predicted position
                    new_azimuth, new_elevation, stepper_steps = move_to_polar_position(
                        pi, azimuth_deg, elevation_deg, stepper_steps
                    )
                    
                    print(f"Motors moved to position: {new_azimuth:.1f}°, {new_elevation:.1f}°")
                    
                    # Check if drone is approaching for potential intercept
                    intercept_time = tracker.estimate_intercept_time()
                    if 0 < intercept_time < 5:  # Within 5 seconds
                        print(f"*** INTERCEPT OPPORTUNITY in {intercept_time:.1f} seconds! ***")
                        
                        # Get trajectory for precise targeting
                        trajectory = tracker.get_trajectory_points(intercept_time + 1, 0.1)
                        closest_point = min(trajectory, key=lambda p: math.sqrt(p[1]**2 + p[2]**2 + p[3]**2))
                        
                        print(f"Closest approach at t+{closest_point[0]:.1f}s: "
                              f"{closest_point[4]:.0f}cm, {closest_point[5]:.1f}°, {closest_point[6]:.1f}°")
                        
                        # Move to intercept position
                        intercept_azimuth, intercept_elevation, stepper_steps = move_to_polar_position(
                            pi, closest_point[5], closest_point[6], stepper_steps
                        )
                        
                        print("MOVED TO INTERCEPT POSITION!")
                        
                        # Wait for intercept (you could add additional logic here)
                        time.sleep(intercept_time)
                        print("INTERCEPT TIME REACHED!")
                        
                        return False  # End tracking
                    
                except Exception as e:
                    print(f"Error in prediction/movement: {e}")
                    print("Continuing with current position...")
            
            last_move_time = current_time
        
        # Brief pause to prevent overwhelming the system
        time.sleep(0.01)
    
    print("Tracking duration completed")
    return False

def display_trajectory_analysis(tracker):
    """
    Display detailed trajectory analysis and predictions.
    
    Args:
        tracker: DroneEKF instance with measurement data
    """
    print("\n=== TRAJECTORY ANALYSIS ===")
    print(f"Total measurements: {tracker.get_measurement_count()}")
    print(f"Tracking confidence: {tracker.get_tracking_confidence():.2f}")
    print(f"Current speed: {tracker.get_current_velocity_magnitude():.2f} m/s")
    
    current_pos = tracker.x[:3]
    current_vel = tracker.x[3:6]
    
    print(f"Current position: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f}) m")
    print(f"Current velocity: ({current_vel[0]:.2f}, {current_vel[1]:.2f}, {current_vel[2]:.2f}) m/s")
    
    # Show future trajectory
    print("\n=== PREDICTED TRAJECTORY ===")
    trajectory = tracker.get_trajectory_points(10, 1.0)
    
    for time_point, x, y, z, dist_cm, az_deg, el_deg in trajectory[:6]:
        print(f"t+{time_point:4.1f}s: Position ({x:6.2f}, {y:6.2f}, {z:6.2f})m "
              f"-> Motors ({az_deg:6.1f}°, {el_deg:5.1f}°) Distance {dist_cm:6.0f}cm")
    
    # Intercept analysis
    intercept_time = tracker.estimate_intercept_time()
    if intercept_time > 0:
        print(f"\nClosest approach in {intercept_time:.1f} seconds")
        intercept_pos = tracker.predict_future_position(intercept_time)
        intercept_dist = math.sqrt(intercept_pos[0]**2 + intercept_pos[1]**2 + intercept_pos[2]**2)
        print(f"Closest distance: {intercept_dist:.2f} meters")
    else:
        print(f"\nDrone already passed closest point {-intercept_time:.1f} seconds ago")


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

    try:
        while True:
            if current_state == "CALIBRATING":
                print("Calibrating sensors...")

                # Mapping environment
                calibration_data = calibrate_environment(pi, lidar_data_queue)
                print(calibration_data)
                save_calibration_data(calibration_data)

                # Moving to right of ascending node
                current_azimuth, current_elevation, stepper_steps = move_to_polar_position(pi, tle_data["arg_perigee_deg"], 30 , stepper_steps)

                calibration_done = True
                if calibration_done:
                    current_state = states[1]

            # Corrected scanning section for main.py

            elif current_state == "SCANNING":
                print("Scanning area...")
                
                current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, should_change_state = perform_scanning_sequence(
                    pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                    stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count
                )
                
                print(anomaly_averaged_coords)
                if should_change_state:
                    current_state = states[2]

            elif current_state == "DETECTED":
                print("=== TARGET DETECTED - INITIALIZING EKF TRACKING ===")
                
                # Initialize EKF tracker with first detection as starting point
                if len(anomaly_averaged_coords) > 0:
                    first_detection = anomaly_averaged_coords[0]
                    if len(first_detection) >= 4:  # [distance, azimuth, elevation, timestamp]
                        initial_measurement = first_detection[:3]
                        tracker = create_drone_tracker(initial_detection=initial_measurement)
                        print(f"EKF initialized with first detection: {initial_measurement}")
                    else:
                        tracker = create_drone_tracker()
                        print("EKF initialized without specific initial detection")
                else:
                    tracker = create_drone_tracker()
                    print("EKF initialized - no prior detections available")
                
                # Update tracker with all existing anomaly detections
                if len(anomaly_averaged_coords) > 0:
                    print(f"Updating EKF with {len(anomaly_averaged_coords)} existing detections...")
                    for i, coords_group in enumerate(anomaly_averaged_coords):
                        if len(coords_group) >= 4:
                            distance, azimuth, elevation, timestamp = coords_group[:4]
                            
                            # Convert timestamp to datetime if needed
                            if isinstance(timestamp, (int, float)):
                                dt_timestamp = datetime.fromtimestamp(timestamp)
                            else:
                                dt_timestamp = timestamp
                            
                            measurement = [distance, azimuth, elevation]
                            tracker.update(measurement, dt_timestamp)
                            print(f"  Detection {i+1}: {distance:.1f}cm at ({azimuth:.1f}°, {elevation:.1f}°)")
                
                print(f"EKF updated with {tracker.get_measurement_count()} total measurements")
                print(f"Initial tracking confidence: {tracker.get_tracking_confidence():.2f}")
                
                # Only proceed with predictive tracking if we have enough data
                if tracker.get_measurement_count() < 3:
                    print("Insufficient measurements for reliable prediction - collecting more data...")
                    
                    # Continue scanning to get more measurements
                    current_azimuth, current_elevation, stepper_steps, anomaly_averaged_coords, anomaly_count, should_continue = perform_scanning_sequence(
                        pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                        stepper_steps, anomaly_locations, anomaly_averaged_coords, anomaly_count
                    )
                    
                    # Update tracker with any new detections
                    if len(anomaly_averaged_coords) > tracker.get_measurement_count():
                        new_detections = anomaly_averaged_coords[tracker.get_measurement_count():]
                        for coords_group in new_detections:
                            if len(coords_group) >= 4:
                                distance, azimuth, elevation, timestamp = coords_group[:4]
                                if isinstance(timestamp, (int, float)):
                                    dt_timestamp = datetime.fromtimestamp(timestamp)
                                else:
                                    dt_timestamp = timestamp
                                
                                measurement = [distance, azimuth, elevation]
                                tracker.update(measurement, dt_timestamp)
                    
                    print(f"Updated tracking confidence: {tracker.get_tracking_confidence():.2f}")
                
                # Main predictive tracking loop
                if tracker.get_measurement_count() >= 3:
                    print("=== STARTING PREDICTIVE TRACKING ===")
                    
                    tracking_duration = 30.0  # Track for 30 seconds
                    prediction_time = 1.5     # Predict 1.5 seconds ahead
                    update_interval = 0.3     # Update motors every 0.3 seconds
                    
                    start_time = datetime.now()
                    last_move_time = datetime.now()
                    consecutive_no_detection = 0
                    max_no_detection = 10  # Stop tracking if no detection for this many cycles
                    
                    print(f"Tracking parameters: Duration={tracking_duration}s, Prediction={prediction_time}s, Update={update_interval}s")
                    
                    while (datetime.now() - start_time).total_seconds() < tracking_duration:
                        current_time = datetime.now()
                        
                        # Check for new LiDAR measurements
                        new_detection = False
                        try:
                            distance = lidar_data_queue.get_nowait()
                            
                            # Get current motor positions for anomaly checking
                            current_azimuth_est = (stepper_steps / STEPS_PER_REVOLUTION) * 360
                            
                            # Get reference distance for anomaly detection
                            reference = get_interpolated_reference_distance(
                                current_azimuth_est, current_elevation, calibration_data
                            )
                            
                            # Check if this is an anomaly (potential drone detection)
                            if distance < reference * ANOMALY_FACTOR:
                                # New detection - update EKF
                                measurement = [distance, current_azimuth_est, current_elevation]
                                tracker.update(measurement, current_time)
                                
                                print(f"New detection: {distance:.1f}cm at ({current_azimuth_est:.1f}°, {current_elevation:.1f}°)")
                                print(f"Tracking confidence: {tracker.get_tracking_confidence():.2f}")
                                
                                new_detection = True
                                consecutive_no_detection = 0
                                
                                # Validate tracking quality
                                if not tracker.validate_velocity_direction():
                                    print("Velocity direction validation failed - recalibrating...")
                                    tracker.recalibrate_velocity()
                            
                        except queue.Empty:
                            pass
                        
                        # Count consecutive cycles without detection
                        if not new_detection:
                            consecutive_no_detection += 1
                            if consecutive_no_detection >= max_no_detection:
                                print(f"No detections for {consecutive_no_detection} cycles - target likely lost")
                                break
                        
                        # Move motors to predicted position periodically
                        if (current_time - last_move_time).total_seconds() >= update_interval:
                            try:
                                # Predict future position
                                future_distance, future_azimuth, future_elevation = tracker.predict_future_spherical(prediction_time)
                                
                                # Validate prediction reasonableness
                                current_pos = tracker.x[:3]
                                current_distance = math.sqrt(current_pos[0]**2 + current_pos[1]**2 + current_pos[2]**2) * 100
                                distance_change = abs(future_distance - current_distance)
                                max_reasonable_change = tracker.get_current_velocity_magnitude() * prediction_time * 100 * 2.0
                                
                                if distance_change > max_reasonable_change and max_reasonable_change > 0:
                                    print(f"Warning: Predicted distance change ({distance_change:.0f}cm) exceeds reasonable limit ({max_reasonable_change:.0f}cm)")
                                    print("Using shorter prediction time...")
                                    future_distance, future_azimuth, future_elevation = tracker.predict_future_spherical(prediction_time * 0.5)
                                
                                print(f"Predicting target at t+{prediction_time:.1f}s: {future_distance:.0f}cm, {future_azimuth:.1f}°, {future_elevation:.1f}°")
                                
                                # Move motors to predicted position
                                actual_azimuth, actual_elevation, stepper_steps = move_to_polar_position(
                                    pi, future_azimuth, future_elevation, stepper_steps
                                )
                                
                                current_azimuth = actual_azimuth
                                current_elevation = actual_elevation
                                
                                print(f"Motors positioned at: {actual_azimuth:.1f}°, {actual_elevation:.1f}°")
                                
                                # Check for intercept opportunity
                                intercept_time = tracker.estimate_intercept_time()
                                if 0 < intercept_time < 5.0:  # Intercept within 5 seconds
                                    print(f"*** INTERCEPT OPPORTUNITY in {intercept_time:.1f} seconds! ***")
                                    
                                    # Calculate intercept position
                                    intercept_pos = tracker.predict_future_position(intercept_time)
                                    intercept_azimuth, intercept_elevation, intercept_distance = xyz_to_polar(
                                        intercept_pos[0], intercept_pos[1], intercept_pos[2]
                                    )
                                    
                                    print(f"Moving to intercept position: {intercept_azimuth:.1f}°, {intercept_elevation:.1f}°")
                                    
                                    # Move to intercept position
                                    actual_azimuth, actual_elevation, stepper_steps = move_to_polar_position(
                                        pi, intercept_azimuth, intercept_elevation, stepper_steps
                                    )
                                    
                                    print("*** INTERCEPT POSITION REACHED ***")
                                    
                                    # Wait for intercept
                                    time.sleep(max(0, intercept_time - 0.1))  # Account for movement time
                                    print("*** INTERCEPT TIME! ***")
                                    
                                    # Show final trajectory analysis
                                    display_trajectory_analysis(tracker)
                                    
                                    # End tracking after intercept
                                    break
                                    
                            except Exception as e:
                                print(f"Error in prediction/movement: {e}")
                                print("Continuing with current position...")
                            
                            last_move_time = current_time
                        
                        # Brief pause to prevent system overload
                        time.sleep(0.01)
                    
                    # Display final tracking results
                    print("\n=== TRACKING COMPLETED ===")
                    display_trajectory_analysis(tracker)
                    
                else:
                    print("Insufficient measurements for reliable tracking")
                
                # Return to scanning state to look for new targets
                print("Returning to scanning mode...")
                current_state = states[1]
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