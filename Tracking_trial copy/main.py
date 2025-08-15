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
from trajectory_prediction import TrajectoryPredictor

import requests
import json

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
    states = ["CALIBRATING", "SCANNING", "DETECTED", "TRACKING"]
    current_state = states[0]
    stepper_steps = 0
    current_azimuth = 0
    current_elevation = 0
    anomaly_detected = False
    anomaly_locations = []
    anomaly_count = 0
    anomaly_averaged_coords = []
    
    # Initialize trajectory predictor
    trajectory_predictor = TrajectoryPredictor()
    last_update_time = time.time()
    
    def send_to_ui(point_data):
        """Send point data to UI via API"""
        try:
            response = requests.post('http://localhost:3000/api/points', json=point_data)
            if response.status_code == 200:
                print("Point data sent to UI successfully")
            else:
                print(f"Failed to send point data to UI: {response.status_code}")
        except Exception as e:
            print(f"Error sending data to UI: {str(e)}")

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
                
                # Lidar queue and data reset
                GPIO.output(DIR_PIN, GPIO.HIGH)
                print("Forward sweep...")
                while not lidar_data_queue.empty():
                    try:
                        lidar_data_queue.get_nowait()
                    except queue.Empty:
                        break
                # Forward sweep
                
                for i in range(round(STEPS_PER_REVOLUTION * SWEEP_RANGE / 360)):
                    # Step the motor first
                    stepper_step()
                    stepper_steps += 1
                    time.sleep(0.0005)

                    # Calculate current azimuth position (incremental update)
                    current_azimuth = (stepper_steps / STEPS_PER_REVOLUTION) * 360
                    
                    # Get LiDAR reading
                    try:
                        distance = lidar_data_queue.get_nowait()
                        
                        # Get reference distance for this position
                        reference = get_interpolated_reference_distance(current_azimuth, current_elevation, calibration_data)
                        
                        
                        
                        print(distance - (reference * ANOMALY_FACTOR))
                        
                        
                        
                        # Check for anomaly
                        if distance < reference * ANOMALY_FACTOR:

                            # Store as [distance, azimuth, elevation] triplet
                            anomaly_locations.append([distance, current_azimuth, current_elevation])
                            print(f"Anomaly detected: {distance:.1f}cm at ({current_azimuth:.1f}°, {current_elevation:.1f}°), "
                                f"expected: {reference:.1f}cm, difference: {distance - (reference * ANOMALY_FACTOR):.1f}cm")
                            
                        # Check if we have enough anomalies to declare detection
                        if len(anomaly_locations) >= 3:
                                anomaly_averaged_coords.append([tuple(round(sum(col) / len(col), 2) for col in zip(*anomaly_locations))])
                                anomaly_locations = []
                                current_azimuth, current_elevation, stepper_steps = move_to_polar_position(pi, current_azimuth+10+SWEEP_RANGE, current_elevation , stepper_steps)
                                anomaly_count += 1
   
                    except queue.Empty:
                        continue

                    if anomaly_count >= 3:
                        current_state = states[2]     

                # Reverse sweep
                GPIO.output(DIR_PIN, GPIO.LOW)
                print("Reverse sweep...")
                
                for i in range(round(STEPS_PER_REVOLUTION * SWEEP_RANGE / 360)):
                    # Step the motor first
                    stepper_step()
                    stepper_steps -= 1
                    time.sleep(0.0005)
                    # Calculate current azimuth position
                    current_azimuth = (stepper_steps / STEPS_PER_REVOLUTION) * 360
                    
                    # Get LiDAR reading
                    try:
                        distance = lidar_data_queue.get_nowait()
                        
                        # Get reference distance for this position
                        reference = get_interpolated_reference_distance(current_azimuth, current_elevation, calibration_data)
                        

                        print(distance - (reference * ANOMALY_FACTOR))


                        # Check for anomaly
                        if distance < reference * ANOMALY_FACTOR:
                            # Store as [distance, azimuth, elevation] triplet
                            anomaly_locations.append([distance, current_azimuth, current_elevation])
                            print(f"Anomaly detected: {distance:.1f}cm at ({current_azimuth:.1f}°, {current_elevation:.1f}°), "
                                f"expected: {reference:.1f}cm, difference: {distance - (reference * ANOMALY_FACTOR):.1f}cm")
                            
                        # Check if we have enough anomalies to declare detection
                        if len(anomaly_locations) >= 3:
                            anomaly_averaged_coords.append([tuple(round(sum(col) / len(col), 2) for col in zip(*anomaly_locations))])
                            anomaly_locations = []
                            current_azimuth, current_elevation, stepper_steps = move_to_polar_position(pi, current_azimuth+10+SWEEP_RANGE, current_elevation , stepper_steps)
                            anomaly_count += 1
                            
                    except queue.Empty:
                        continue


                if anomaly_count >= 3:
                    print(anomaly_averaged_coords)
                    current_state = states[2]

            elif current_state == "DETECTED":
                print("Target detected! Starting tracking mode...")
                # Initialize tracking with detected points
                for coords in anomaly_averaged_coords:
                    distance, azimuth, elevation = coords[0]
                    trajectory_predictor.add_point(distance, azimuth, elevation)
                    
                    # Send point to UI
                    point_data = {
                        "distance": distance,
                        "azimuth": azimuth,
                        "elevation": elevation,
                        "timestamp": time.time()
                    }
                    send_to_ui(point_data)
                
                current_state = "TRACKING"
                
            elif current_state == "TRACKING":
                current_time = time.time()
                
                # Get predicted position and tracking window size
                predicted_az, predicted_el = trajectory_predictor.predict_position(PREDICTION_LOOKAHEAD)
                sweep_range, elevation_range = trajectory_predictor.get_tracking_window()
                
                if predicted_az is not None and predicted_el is not None:
                    # Move to predicted position
                    target_az = predicted_az
                    target_el = predicted_el
                    
                    # Perform narrow zigzag scan around predicted position
                    az_start = target_az - sweep_range/2
                    az_end = target_az + sweep_range/2
                    el_start = max(SERVO_SWEEP_START, target_el - elevation_range/2)
                    el_end = min(SERVO_SWEEP_END, target_el + elevation_range/2)
                    
                    # Set initial elevation
                    set_servo_angle(pi, el_start)
                    current_elevation = el_start
                    time.sleep(0.1)  # Allow servo to reach position
                    
                    # Zigzag pattern
                    el_step = (el_end - el_start) / 4  # 4 elevation levels
                    going_up = True
                    
                    for el in [el_start + i * el_step for i in range(4)]:
                        # Set servo elevation
                        set_servo_angle(pi, el)
                        current_elevation = el
                        time.sleep(0.05)
                        
                        # Sweep horizontally
                        steps_to_move = int((sweep_range / 360) * STEPS_PER_REVOLUTION)
                        GPIO.output(DIR_PIN, GPIO.HIGH if going_up else GPIO.LOW)
                        
                        for _ in range(steps_to_move):
                            stepper_step()
                            if going_up:
                                stepper_steps += 1
                            else:
                                stepper_steps -= 1
                            time.sleep(0.0005)
                            
                            # Update current position
                            current_azimuth = (stepper_steps / STEPS_PER_REVOLUTION) * 360
                            
                            # Check for target
                            try:
                                distance = lidar_data_queue.get_nowait()
                                reference = get_interpolated_reference_distance(current_azimuth, current_elevation, calibration_data)
                                
                                if distance < reference * ANOMALY_FACTOR:
                                    # Target detected, update trajectory
                                    trajectory_predictor.add_point(distance, current_azimuth, current_elevation)
                                    
                                    # Send to UI
                                    point_data = {
                                        "distance": distance,
                                        "azimuth": current_azimuth,
                                        "elevation": current_elevation,
                                        "timestamp": time.time()
                                    }
                                    send_to_ui(point_data)
                                    
                                    # Break inner loop to get new prediction
                                    break
                                    
                            except queue.Empty:
                                continue
                                
                        going_up = not going_up
                        
                    if current_time - last_update_time > TRACKING_UPDATE_INTERVAL:
                        last_update_time = current_time
                        # Get new prediction for next iteration
                        continue
                        
                else:
                    print("Lost tracking, attempting to reacquire...")
                    # Could implement reacquisition strategy here
                    time.sleep(0.1)
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        print("Cleaning up...")
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        GPIO.cleanup()
        reset_stepper_pos(stepper_steps)
        

if __name__ == '__main__':
    main()