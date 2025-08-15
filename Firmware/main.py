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
                    # Predict where drone will be in prediction_time seconds
                    future_distance, future_azimuth, future_elevation = tracker.predict_future_spherical(prediction_time)
                    
                    print(f"Predicting drone at t+{prediction_time}s: "
                          f"{future_distance:.0f}cm, {future_azimuth:.1f}°, {future_elevation:.1f}°")
                    
                    # Move motors to predicted position
                    new_azimuth, new_elevation, stepper_steps = move_to_polar_position(
                        pi, future_azimuth, future_elevation, stepper_steps
                    )
                    
                    print(f"Motors moved to predicted position: {new_azimuth:.1f}°, {new_elevation:.1f}°")
                    
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
                print("Target detected! Starting EKF tracking...")
                
                # Use the enhanced detection state with EKF tracking
                continue_tracking = enhanced_detected_state(
                    pi, lidar_data_queue, calibration_data, anomaly_averaged_coords, stepper_steps
                )
                
                if not continue_tracking:
                    print("Tracking completed.")
                    break

                
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