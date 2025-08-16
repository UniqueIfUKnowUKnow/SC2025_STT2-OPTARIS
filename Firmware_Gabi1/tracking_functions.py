# tracking_functions.py
import time
import queue
import numpy as np
from constants import *
from anomaly_check import get_interpolated_reference_distance

class TrackingMetrics:
    """Class to track and analyze tracking performance metrics"""
    
    def __init__(self):
        self.detection_count = 0
        self.missed_detections = 0
        self.prediction_errors = []
        self.tracking_quality_scores = []
        self.last_update_time = time.time()
        self.consecutive_misses = 0
        self.consecutive_hits = 0
        
    def update_detection(self, prediction_error_deg):
        """Update metrics when target is detected"""
        self.detection_count += 1
        self.consecutive_hits += 1
        self.consecutive_misses = 0
        self.prediction_errors.append(prediction_error_deg)
        self.last_update_time = time.time()
        
        # Calculate tracking quality score (0-100)
        if len(self.prediction_errors) >= 3:
            recent_errors = self.prediction_errors[-3:]
            avg_error = np.mean(recent_errors)
            quality_score = max(0, 100 - (avg_error * 10))  # Scale error to 0-100
            self.tracking_quality_scores.append(quality_score)
    
    def update_miss(self):
        """Update metrics when target is missed"""
        self.missed_detections += 1
        self.consecutive_misses += 1
        self.consecutive_hits = 0
        
    def get_tracking_quality(self):
        """Get current tracking quality score"""
        if not self.tracking_quality_scores:
            return 50.0  # Default neutral score
        return self.tracking_quality_scores[-1]
    
    def get_detection_rate(self):
        """Get detection rate as percentage"""
        total_attempts = self.detection_count + self.missed_detections
        if total_attempts == 0:
            return 0.0
        return (self.detection_count / total_attempts) * 100
    
    def get_average_prediction_error(self):
        """Get average prediction error in degrees"""
        if not self.prediction_errors:
            return 0.0
        return np.mean(self.prediction_errors)
    
    def should_adjust_sensitivity(self):
        """Determine if tracking sensitivity should be adjusted"""
        if self.consecutive_misses >= 3:
            return "increase"  # Need more sensitive detection
        elif self.consecutive_hits >= 5 and self.get_average_prediction_error() < 2.0:
            return "decrease"  # Can be less sensitive for efficiency
        return "maintain"

class MultiTargetTracker:
    """Class to handle tracking of multiple targets simultaneously"""
    
    def __init__(self, max_targets=3):
        self.max_targets = max_targets
        self.targets = {}  # target_id -> target_data
        self.next_target_id = 1
        
    def add_target(self, initial_measurement, initial_time):
        """Add a new target to track"""
        if len(self.targets) >= self.max_targets:
            print(f"Warning: Maximum targets ({self.max_targets}) reached")
            return None
            
        target_id = self.next_target_id
        self.next_target_id += 1
        
        self.targets[target_id] = {
            'measurements': [initial_measurement],
            'timestamps': [initial_time],
            'last_seen': initial_time,
            'tracking_quality': 100.0,
            'kalman_filter': None  # Will be initialized when enough measurements
        }
        
        print(f"Added new target {target_id} at position ({initial_measurement[1]:.1f}°, {initial_measurement[2]:.1f}°)")
        return target_id
    
    def update_target(self, target_id, measurement, timestamp):
        """Update an existing target with new measurement"""
        if target_id not in self.targets:
            print(f"Warning: Target {target_id} not found")
            return False
            
        target = self.targets[target_id]
        target['measurements'].append(measurement)
        target['timestamps'].append(timestamp)
        target['last_seen'] = timestamp
        
        # Initialize Kalman filter if we have enough measurements
        if len(target['measurements']) >= 2 and target['kalman_filter'] is None:
            from kalman_filter import DroneTrajectoryKalman
            target['kalman_filter'] = DroneTrajectoryKalman()
            target['kalman_filter'].process_measurement_sequence(
                target['measurements'], target['timestamps']
            )
            print(f"Initialized Kalman filter for target {target_id}")
        
        # Update Kalman filter if available
        elif target['kalman_filter'] is not None:
            target['kalman_filter'].update_with_measurement_at_time(measurement, timestamp)
        
        return True
    
    def get_target_predictions(self, target_id, future_time):
        """Get prediction for a specific target"""
        if target_id not in self.targets or self.targets[target_id]['kalman_filter'] is None:
            return None
            
        try:
            predictions = self.targets[target_id]['kalman_filter'].predict_future_positions([future_time])
            return predictions[0] if predictions else None
        except Exception as e:
            print(f"Error predicting target {target_id}: {e}")
            return None
    
    def remove_stale_targets(self, max_age_seconds=30.0):
        """Remove targets that haven't been seen recently"""
        current_time = time.time()
        stale_targets = []
        
        for target_id, target in self.targets.items():
            if current_time - target['last_seen'] > max_age_seconds:
                stale_targets.append(target_id)
        
        for target_id in stale_targets:
            del self.targets[target_id]
            print(f"Removed stale target {target_id}")
        
        return len(stale_targets)

def detect_at_position(pi, lidar_data_queue, calibration_data, current_azimuth, current_elevation, 
                      max_samples=50, detection_threshold_factor=ANOMALY_FACTOR, adaptive_threshold=True):
    """
    Stay at current position and collect LiDAR samples to detect if target is present.
    Enhanced with adaptive thresholding and better noise handling.
    
    Args:
        pi: pigpio instance 
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data for anomaly detection
        current_azimuth: Current azimuth position in degrees
        current_elevation: Current elevation position in degrees
        max_samples: Maximum number of LiDAR samples to collect
        detection_threshold_factor: Factor to determine if reading is anomalous
        adaptive_threshold: Whether to use adaptive thresholding based on recent performance
        
    Returns:
        tuple: (detection_found, average_distance, sample_count, detection_confidence)
            - detection_found: Boolean indicating if target was detected
            - average_distance: Average distance of detected readings (or None)
            - sample_count: Number of valid samples collected
            - detection_confidence: Confidence score (0-100) for the detection
    """
    
    print(f"Searching at position: Az={current_azimuth:.1f}°, El={current_elevation:.1f}°")
    
    # Clear any old readings from queue
    while not lidar_data_queue.empty():
        try:
            lidar_data_queue.get_nowait()
        except queue.Empty:
            break
    
    # Get reference distance for this position
    reference_distance = get_interpolated_reference_distance(current_azimuth, current_elevation, calibration_data)
    
    # Adaptive threshold adjustment
    if adaptive_threshold:
        # Adjust threshold based on recent tracking performance
        # This could be passed in from the main tracking loop
        detection_threshold = reference_distance * detection_threshold_factor
    else:
        detection_threshold = reference_distance * detection_threshold_factor
    
    print(f"Reference distance: {reference_distance:.1f}cm, Detection threshold: {detection_threshold:.1f}cm")
    
    # Collect samples at this position
    valid_readings = []
    anomaly_readings = []
    sample_count = 0
    start_time = time.time()
    max_wait_time = 2.0  # Maximum time to wait for samples (seconds)
    
    while sample_count < max_samples and (time.time() - start_time) < max_wait_time:
        try:
            # Get LiDAR reading with short timeout
            distance = lidar_data_queue.get(timeout=0.1)
            
            if distance > 0 and distance < SENSOR_MAX:  # Valid reading
                valid_readings.append(distance)
                sample_count += 1
                
                # Check if this reading indicates target presence
                if distance < detection_threshold:
                    anomaly_readings.append(distance)
                    print(f"  Sample {sample_count}: {distance:.1f}cm - ANOMALY DETECTED!")
                else:
                    print(f"  Sample {sample_count}: {distance:.1f}cm - normal")
                
                # Early detection: if we have enough anomaly readings, we can conclude
                if len(anomaly_readings) >= 3:
                    print(f"Target detected with {len(anomaly_readings)} anomalous readings!")
                    avg_distance = sum(anomaly_readings) / len(anomaly_readings)
                    confidence = min(100, len(anomaly_readings) / max_samples * 100)
                    return True, avg_distance, sample_count, confidence
                    
        except queue.Empty:
            # No reading available, continue waiting
            time.sleep(0.05)
            continue
        except Exception as e:
            print(f"Error reading LiDAR: {e}")
            continue
    
    # Analysis of collected samples
    if not valid_readings:
        print("No valid LiDAR readings collected")
        return False, None, 0, 0
    
    # Determine if target is present based on anomaly ratio
    anomaly_ratio = len(anomaly_readings) / len(valid_readings) if valid_readings else 0
    detection_found = anomaly_ratio >= 0.3  # Need at least 30% anomalous readings
    
    # Calculate detection confidence
    if detection_found:
        confidence = min(100, anomaly_ratio * 100)
        avg_distance = sum(anomaly_readings) / len(anomaly_readings)
        print(f"Target detected! {len(anomaly_readings)}/{len(valid_readings)} readings were anomalous")
        print(f"Average target distance: {avg_distance:.1f}cm, Confidence: {confidence:.1f}%")
        return True, avg_distance, sample_count, confidence
    else:
        confidence = max(0, (1 - anomaly_ratio) * 100)
        print(f"No target detected. {len(anomaly_readings)}/{len(valid_readings)} readings were anomalous (need ≥30%)")
        print(f"Confidence in no-target: {confidence:.1f}%")
        return False, None, sample_count, confidence


def perform_local_search(pi, lidar_data_queue, calibration_data, center_azimuth, center_elevation, 
                        stepper_steps, search_radius_deg=5.0):
    """
    Perform a small local search around the predicted position to account for prediction errors.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings  
        calibration_data: Reference calibration data
        center_azimuth: Center azimuth position for search
        center_elevation: Center elevation position for search
        stepper_steps: Current stepper step count
        search_radius_deg: Radius in degrees to search around center position
        
    Returns:
        tuple: (detection_found, best_measurement, final_azimuth, final_elevation, final_stepper_steps)
    """
    
    print(f"Performing local search around Az={center_azimuth:.1f}°, El={center_elevation:.1f}°")
    print(f"Search radius: ±{search_radius_deg:.1f}°")
    
    from move_motors import move_to_polar_position
    
    # Define search pattern (spiral outward from center)
    search_positions = [
        (0, 0),  # Center position first
        (-search_radius_deg, 0), (search_radius_deg, 0),  # Left/right
        (0, -search_radius_deg), (0, search_radius_deg),  # Down/up  
        (-search_radius_deg, -search_radius_deg), (search_radius_deg, search_radius_deg),  # Diagonals
        (search_radius_deg, -search_radius_deg), (-search_radius_deg, search_radius_deg)
    ]
    
    best_detection = None
    best_measurement = None
    
    for i, (az_offset, el_offset) in enumerate(search_positions):
        search_azimuth = center_azimuth + az_offset
        search_elevation = center_elevation + el_offset
        
        # Clamp elevation to servo limits
        search_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, search_elevation))
        
        print(f"\nSearch position {i+1}/{len(search_positions)}: Az={search_azimuth:.1f}°, El={search_elevation:.1f}°")
        
        # Move to search position
        actual_az, actual_el, stepper_steps = move_to_polar_position(pi, search_azimuth, search_elevation, stepper_steps)
        
        # Wait for motors to settle
        time.sleep(0.3)
        
        # Search for target at this position
        detection_found, avg_distance, sample_count = detect_at_position(
            pi, lidar_data_queue, calibration_data, actual_az, actual_el
        )
        
        if detection_found:
            measurement = [avg_distance, actual_az, actual_el, time.time()]
            
            # Use first detection found, or keep the one with shortest distance (closest target)
            if best_detection is None or avg_distance < best_measurement[0]:
                best_detection = True
                best_measurement = measurement
                print(f"*** BEST DETECTION SO FAR: {avg_distance:.1f}cm at ({actual_az:.1f}°, {actual_el:.1f}°) ***")
            
            # For efficiency, you could break here if you only want the first detection
            # break
    
    # Return to center position if no detection found
    if best_detection is None:
        print("No target found in local search, returning to center position")
        actual_az, actual_el, stepper_steps = move_to_polar_position(pi, center_azimuth, center_elevation, stepper_steps)
        return False, None, actual_az, actual_el, stepper_steps
    else:
        # Move to best detection position
        best_az, best_el = best_measurement[1], best_measurement[2]
        actual_az, actual_el, stepper_steps = move_to_polar_position(pi, best_az, best_el, stepper_steps)
        print(f"Moving to best detection position: Az={best_az:.1f}°, El={best_el:.1f}°")
        return True, best_measurement, actual_az, actual_el, stepper_steps


def perform_adaptive_search(pi, lidar_data_queue, calibration_data, center_azimuth, center_elevation, 
                           stepper_steps, search_radius_deg=5.0, tracking_metrics=None):
    """
    Perform adaptive local search that adjusts search pattern based on tracking performance.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings  
        calibration_data: Reference calibration data
        center_azimuth: Center azimuth position for search
        center_elevation: Center elevation position for search
        stepper_steps: Current stepper step count
        search_radius_deg: Initial radius in degrees to search around center position
        tracking_metrics: Optional TrackingMetrics instance for adaptive behavior
        
    Returns:
        tuple: (detection_found, best_measurement, final_azimuth, final_elevation, final_stepper_steps)
    """
    
    print(f"Performing adaptive search around Az={center_azimuth:.1f}°, El={center_elevation:.1f}°")
    
    # Adjust search parameters based on tracking performance
    if tracking_metrics:
        quality = tracking_metrics.get_tracking_quality()
        if quality < 30:  # Poor tracking - expand search
            search_radius_deg *= 1.5
            print(f"Poor tracking quality ({quality:.1f}), expanding search radius to {search_radius_deg:.1f}°")
        elif quality > 80:  # Good tracking - tighten search
            search_radius_deg *= 0.7
            print(f"Good tracking quality ({quality:.1f}), tightening search radius to {search_radius_deg:.1f}°")
    
    print(f"Search radius: ±{search_radius_deg:.1f}°")
    
    from move_motors import move_to_polar_position
    
    # Define adaptive search pattern (spiral outward from center)
    search_positions = [
        (0, 0),  # Center position first
        (-search_radius_deg * 0.5, 0), (search_radius_deg * 0.5, 0),  # Close left/right
        (0, -search_radius_deg * 0.5), (0, search_radius_deg * 0.5),  # Close down/up
        (-search_radius_deg, 0), (search_radius_deg, 0),  # Full left/right
        (0, -search_radius_deg), (0, search_radius_deg),  # Full down/up
        (-search_radius_deg, -search_radius_deg), (search_radius_deg, search_radius_deg),  # Diagonals
        (search_radius_deg, -search_radius_deg), (-search_radius_deg, search_radius_deg)
    ]
    
    best_detection = None
    best_measurement = None
    best_confidence = 0
    
    for i, (az_offset, el_offset) in enumerate(search_positions):
        search_azimuth = center_azimuth + az_offset
        search_elevation = center_elevation + el_offset
        
        # Clamp elevation to servo limits
        search_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, search_elevation))
        
        print(f"\nSearch position {i+1}/{len(search_positions)}: Az={search_azimuth:.1f}°, El={search_elevation:.1f}°")
        
        # Move to search position
        actual_az, actual_el, stepper_steps = move_to_polar_position(pi, search_azimuth, search_elevation, stepper_steps)
        
        # Wait for motors to settle
        time.sleep(0.3)
        
        # Search for target at this position with confidence scoring
        detection_found, avg_distance, sample_count, confidence = detect_at_position(
            pi, lidar_data_queue, calibration_data, actual_az, actual_el
        )
        
        if detection_found:
            measurement = [avg_distance, actual_az, actual_el, time.time()]
            
            # Keep the detection with highest confidence
            if best_detection is None or confidence > best_confidence:
                best_detection = True
                best_measurement = measurement
                best_confidence = confidence
                print(f"*** BEST DETECTION SO FAR: {avg_distance:.1f}cm at ({actual_az:.1f}°, {actual_el:.1f}°) with {confidence:.1f}% confidence ***")
            
            # Early termination if we have very high confidence
            if confidence > 90:
                print(f"High confidence detection ({confidence:.1f}%), terminating search early")
                break
    
    # Return to center position if no detection found
    if best_detection is None:
        print("No target found in adaptive search, returning to center position")
        actual_az, actual_el, stepper_steps = move_to_polar_position(pi, center_azimuth, center_elevation, stepper_steps)
        return False, None, actual_az, actual_el, stepper_steps
    else:
        # Move to best detection position
        best_az, best_el = best_measurement[1], best_measurement[2]
        actual_az, actual_el, stepper_steps = move_to_polar_position(pi, best_az, best_el, stepper_steps)
        print(f"Moving to best detection position: Az={best_az:.1f}°, El={best_el:.1f}° (confidence: {best_confidence:.1f}%)")
        return True, best_measurement, actual_az, actual_el, stepper_steps

def save_tracking_state(tracking_metrics, multi_target_tracker, filename="tracking_state.json"):
    """Save current tracking state to file for persistence"""
    import json
    import os
    
    state_data = {
        'timestamp': time.time(),
        'tracking_metrics': {
            'detection_count': tracking_metrics.detection_count,
            'missed_detections': tracking_metrics.missed_detections,
            'tracking_quality': tracking_metrics.get_tracking_quality(),
            'detection_rate': tracking_metrics.get_detection_rate(),
            'average_prediction_error': tracking_metrics.get_average_prediction_error()
        },
        'targets': {}
    }
    
    # Save target information
    for target_id, target in multi_target_tracker.targets.items():
        state_data['targets'][str(target_id)] = {
            'last_seen': target['last_seen'],
            'tracking_quality': target['tracking_quality'],
            'measurement_count': len(target['measurements'])
        }
    
    try:
        with open(filename, 'w') as f:
            json.dump(state_data, f, indent=2)
        print(f"Tracking state saved to {filename}")
        return True
    except Exception as e:
        print(f"Error saving tracking state: {e}")
        return False

def load_tracking_state(filename="tracking_state.json"):
    """Load tracking state from file"""
    import json
    import os
    
    if not os.path.exists(filename):
        print(f"Tracking state file {filename} not found")
        return None
    
    try:
        with open(filename, 'r') as f:
            state_data = json.load(f)
        print(f"Tracking state loaded from {filename}")
        return state_data
    except Exception as e:
        print(f"Error loading tracking state: {e}")
        return None

def perform_tracking_detection(pi, lidar_data_queue, calibration_data, predicted_azimuth, predicted_elevation, 
                              stepper_steps, tracking_metrics=None, adaptive_search=True):
    """
    Enhanced target detection at predicted location for Kalman tracking.
    Now includes adaptive search and performance metrics.
    
    Args:
        pi: pigpio instance
        lidar_data_queue: Queue containing LiDAR readings
        calibration_data: Reference calibration data
        predicted_azimuth: Predicted azimuth from Kalman filter
        predicted_elevation: Predicted elevation from Kalman filter  
        stepper_steps: Current stepper step count
        tracking_metrics: Optional TrackingMetrics instance for performance tracking
        adaptive_search: Whether to use adaptive search parameters
        
    Returns:
        tuple: (new_measurement, final_azimuth, final_elevation, final_stepper_steps, detection_confidence)
            - new_measurement: [distance, azimuth, elevation, timestamp] if found, None otherwise
            - final_azimuth: Final azimuth position
            - final_elevation: Final elevation position
            - final_stepper_step_count: Final stepper step count
            - detection_confidence: Confidence score for the detection (0-100)
    """
    
    from move_motors import move_to_polar_position
    from constants import SERVO_SWEEP_START, SERVO_SWEEP_END
    
    print(f"=== ENHANCED TRACKING DETECTION ===")
    print(f"Predicted position: Az={predicted_azimuth:.1f}°, El={predicted_elevation:.1f}°")
    print(f"Servo elevation limits: {SERVO_SWEEP_START}° to {SERVO_SWEEP_END}°")
    
    # Validate elevation prediction
    if predicted_elevation < SERVO_SWEEP_START or predicted_elevation > SERVO_SWEEP_END:
        print(f"ERROR: Predicted elevation {predicted_elevation:.1f}° is outside servo range!")
        print(f"Adjusting elevation to valid range...")
        predicted_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, predicted_elevation))
        print(f"Adjusted elevation to: {predicted_elevation:.1f}°")
    
    print(f"Moving to predicted position: Az={predicted_azimuth:.1f}°, El={predicted_elevation:.1f}°")
    
    # Move to predicted position
    actual_azimuth, actual_elevation, stepper_steps = move_to_polar_position(
        pi, predicted_azimuth, predicted_elevation, stepper_steps
    )
    
    print(f"Actually moved to: Az={actual_azimuth:.1f}°, El={actual_elevation:.1f}°")
    
    # Wait for motors to settle
    time.sleep(0.5)
    
    # Try detection at exact predicted position first
    detection_found, avg_distance, sample_count, confidence = detect_at_position(
        pi, lidar_data_queue, calibration_data, actual_azimuth, actual_elevation
    )
    
    if detection_found:
        print(f"Target found at predicted position with {confidence:.1f}% confidence!")
        measurement = [avg_distance, actual_azimuth, actual_elevation, time.time()]
        
        # Update tracking metrics if available
        if tracking_metrics:
            prediction_error = np.sqrt((predicted_azimuth - actual_azimuth)**2 + 
                                     (predicted_elevation - actual_elevation)**2)
            tracking_metrics.update_detection(prediction_error)
        
        return measurement, actual_azimuth, actual_elevation, stepper_steps, confidence
    
    # If not found at exact position, perform adaptive search
    print("Target not found at exact predicted position, performing adaptive search...")
    
    if adaptive_search:
        detection_found, best_measurement, final_az, final_el, stepper_steps = perform_adaptive_search(
            pi, lidar_data_queue, calibration_data, actual_azimuth, actual_elevation, 
            stepper_steps, tracking_metrics=tracking_metrics
        )
    else:
        detection_found, best_measurement, final_az, final_el, stepper_steps = perform_local_search(
            pi, lidar_data_queue, calibration_data, actual_azimuth, actual_elevation, stepper_steps
        )
    
    if detection_found:
        print(f"Target found during search!")
        
        # Update tracking metrics if available
        if tracking_metrics:
            prediction_error = np.sqrt((predicted_azimuth - final_az)**2 + 
                                     (predicted_elevation - final_el)**2)
            tracking_metrics.update_detection(prediction_error)
        
        # Estimate confidence for search-based detection
        search_confidence = 75.0  # Base confidence for search detection
        return best_measurement, final_az, final_el, stepper_steps, search_confidence
    else:
        print("Target not found in search area")
        
        # Update tracking metrics if available
        if tracking_metrics:
            tracking_metrics.update_miss()
        
        return None, final_az, final_el, stepper_steps, 0

def get_tracking_summary(tracking_metrics, multi_target_tracker):
    """Generate a comprehensive tracking performance summary"""
    if not tracking_metrics:
        return "No tracking metrics available"
    
    summary = f"""
=== TRACKING PERFORMANCE SUMMARY ===
Overall Performance:
  - Detection Rate: {tracking_metrics.get_detection_rate():.1f}%
  - Tracking Quality: {tracking_metrics.get_tracking_quality():.1f}/100
  - Average Prediction Error: {tracking_metrics.get_average_prediction_error():.2f}°
  - Total Detections: {tracking_metrics.detection_count}
  - Missed Detections: {tracking_metrics.missed_detections}
  - Consecutive Hits: {tracking_metrics.consecutive_hits}
  - Consecutive Misses: {tracking_metrics.consecutive_misses}

Sensitivity Recommendation: {tracking_metrics.should_adjust_sensitivity()}

Active Targets: {len(multi_target_tracker.targets)}
"""
    
    for target_id, target in multi_target_tracker.targets.items():
        age_seconds = time.time() - target['last_seen']
        summary += f"  Target {target_id}: {len(target['measurements'])} measurements, last seen {age_seconds:.1f}s ago\n"
    
    return summary

def create_tracking_dashboard(tracking_metrics, multi_target_tracker, plot_data=None):
    """
    Create a real-time tracking dashboard with performance metrics and visualizations.
    
    Args:
        tracking_metrics: TrackingMetrics instance
        multi_target_tracker: MultiTargetTracker instance
        plot_data: Optional list of tracking measurements for visualization
        
    Returns:
        str: Formatted dashboard string
    """
    import time
    
    current_time = time.time()
    
    dashboard = f"""
{'='*80}
                    REAL-TIME TRACKING DASHBOARD
{'='*80}
Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(current_time))}

{'='*50}
PERFORMANCE METRICS
{'='*50}
Detection Rate:     {tracking_metrics.get_detection_rate():>6.1f}%
Tracking Quality:   {tracking_metrics.get_tracking_quality():>6.1f}/100
Prediction Error:   {tracking_metrics.get_average_prediction_error():>6.2f}°
Total Detections:   {tracking_metrics.detection_count:>6d}
Missed Detections:  {tracking_metrics.missed_detections:>6d}
Consecutive Hits:   {tracking_metrics.consecutive_hits:>6d}
Consecutive Misses: {tracking_metrics.consecutive_misses:>6d}

{'='*50}
TARGET STATUS
{'='*50}
Active Targets: {len(multi_target_tracker.targets)}
"""
    
    if multi_target_tracker.targets:
        dashboard += f"{'ID':<4} {'Age(s)':<8} {'Measurements':<12} {'Quality':<8}\n"
        dashboard += "-" * 40 + "\n"
        
        for target_id, target in multi_target_tracker.targets.items():
            age_seconds = current_time - target['last_seen']
            measurement_count = len(target['measurements'])
            quality = target['tracking_quality']
            
            dashboard += f"{target_id:<4} {age_seconds:<8.1f} {measurement_count:<12} {quality:<8.1f}\n"
    else:
        dashboard += "No active targets\n"
    
    dashboard += f"\n{'='*50}\n"
    dashboard += f"RECOMMENDATION: {tracking_metrics.should_adjust_sensitivity().upper()}\n"
    dashboard += f"{'='*50}\n"
    
    if plot_data and len(plot_data) > 0:
        dashboard += f"\nTracking Data Points: {len(plot_data)}\n"
        if len(plot_data) >= 3:
            recent_points = plot_data[-3:]
            dashboard += "Recent Positions:\n"
            for i, point in enumerate(recent_points):
                dashboard += f"  {len(plot_data)-2+i}: ({point[1]:.1f}°, {point[2]:.1f}°) at {point[0]:.1f}cm\n"
    
    return dashboard

def visualize_tracking_performance(tracking_metrics, save_plot=True, filename="tracking_performance.png"):
    """
    Create visualization plots for tracking performance analysis.
    
    Args:
        tracking_metrics: TrackingMetrics instance
        save_plot: Whether to save the plot to file
        filename: Filename for saved plot
        
    Returns:
        matplotlib figure object
    """
    try:
        import matplotlib.pyplot as plt
        import matplotlib.dates as mdates
        from datetime import datetime, timedelta
    except ImportError:
        print("Matplotlib not available for visualization")
        return None
    
    if not tracking_metrics.tracking_quality_scores:
        print("No tracking quality data available for visualization")
        return None
    
    # Create figure with subplots
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Tracking Performance Analysis', fontsize=16)
    
    # Plot 1: Tracking Quality Over Time
    timestamps = [datetime.now() - timedelta(seconds=len(tracking_metrics.tracking_quality_scores)-i-1) 
                  for i in range(len(tracking_metrics.tracking_quality_scores))]
    
    ax1.plot(timestamps, tracking_metrics.tracking_quality_scores, 'b-o', linewidth=2, markersize=6)
    ax1.set_title('Tracking Quality Over Time')
    ax1.set_ylabel('Quality Score (0-100)')
    ax1.set_ylim(0, 100)
    ax1.grid(True, alpha=0.3)
    ax1.xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    ax1.tick_params(axis='x', rotation=45)
    
    # Plot 2: Prediction Errors
    if tracking_metrics.prediction_errors:
        ax2.plot(tracking_metrics.prediction_errors, 'r-o', linewidth=2, markersize=6)
        ax2.set_title('Prediction Errors Over Time')
        ax2.set_ylabel('Error (degrees)')
        ax2.set_xlabel('Detection Number')
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=tracking_metrics.get_average_prediction_error(), color='g', linestyle='--', 
                    label=f'Average: {tracking_metrics.get_average_prediction_error():.2f}°')
        ax2.legend()
    
    # Plot 3: Detection Rate
    detection_rate = tracking_metrics.get_detection_rate()
    ax3.bar(['Detection Rate'], [detection_rate], color='green' if detection_rate > 70 else 'orange' if detection_rate > 50 else 'red')
    ax3.set_title('Overall Detection Rate')
    ax3.set_ylabel('Percentage (%)')
    ax3.set_ylim(0, 100)
    ax3.text(0, detection_rate + 2, f'{detection_rate:.1f}%', ha='center', va='bottom', fontsize=12)
    
    # Plot 4: Performance Summary
    metrics_text = f"""
Performance Summary:
• Total Attempts: {tracking_metrics.detection_count + tracking_metrics.missed_detections}
• Successful Detections: {tracking_metrics.detection_count}
• Failed Detections: {tracking_metrics.missed_detections}
• Current Quality: {tracking_metrics.get_tracking_quality():.1f}/100
• Avg Prediction Error: {tracking_metrics.get_average_prediction_error():.2f}°
• Consecutive Hits: {tracking_metrics.consecutive_hits}
• Consecutive Misses: {tracking_metrics.consecutive_misses}
"""
    
    ax4.text(0.1, 0.9, metrics_text, transform=ax4.transAxes, fontsize=10, 
             verticalalignment='top', fontfamily='monospace')
    ax4.set_title('Performance Metrics')
    ax4.axis('off')
    
    plt.tight_layout()
    
    if save_plot:
        try:
            plt.savefig(filename, dpi=300, bbox_inches='tight')
            print(f"Tracking performance plot saved to {filename}")
        except Exception as e:
            print(f"Error saving plot: {e}")
    
    return fig

def export_tracking_data(tracking_metrics, multi_target_tracker, plot_data, filename="tracking_data.csv"):
    """
    Export tracking data to CSV format for external analysis.
    
    Args:
        tracking_metrics: TrackingMetrics instance
        multi_target_tracker: MultiTargetTracker instance
        plot_data: List of tracking measurements
        filename: Output CSV filename
        
    Returns:
        bool: True if export successful, False otherwise
    """
    try:
        import csv
        import os
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            writer.writerow(['Data Type', 'Timestamp', 'Value', 'Details'])
            
            # Write tracking metrics
            current_time = time.time()
            writer.writerow(['Detection Count', current_time, tracking_metrics.detection_count, 'Total successful detections'])
            writer.writerow(['Missed Detections', current_time, tracking_metrics.missed_detections, 'Total failed detections'])
            writer.writerow(['Detection Rate', current_time, tracking_metrics.get_detection_rate(), 'Percentage success rate'])
            writer.writerow(['Tracking Quality', current_time, tracking_metrics.get_tracking_quality(), 'Current quality score'])
            writer.writerow(['Avg Prediction Error', current_time, tracking_metrics.get_average_prediction_error(), 'Average error in degrees'])
            
            # Write target information
            for target_id, target in multi_target_tracker.targets.items():
                writer.writerow(['Target Info', current_time, target_id, f"Target {target_id}: {len(target['measurements'])} measurements"])
            
            # Write tracking measurements
            if plot_data:
                for i, measurement in enumerate(plot_data):
                    timestamp = current_time - (len(plot_data) - i - 1) * DT  # Estimate timestamp
                    writer.writerow(['Measurement', timestamp, f"{measurement[0]:.1f}cm", 
                                   f"Az: {measurement[1]:.1f}°, El: {measurement[2]:.1f}°"])
        
        print(f"Tracking data exported to {filename}")
        return True
        
    except Exception as e:
        print(f"Error exporting tracking data: {e}")
        return False

def adaptive_tracking_parameters(tracking_metrics, base_params):
    """
    Dynamically adjust tracking parameters based on performance metrics.
    
    Args:
        tracking_metrics: TrackingMetrics instance
        base_params: Dictionary of base tracking parameters
        
    Returns:
        dict: Adjusted tracking parameters
    """
    if not tracking_metrics:
        return base_params
    
    quality = tracking_metrics.get_tracking_quality()
    detection_rate = tracking_metrics.get_detection_rate()
    avg_error = tracking_metrics.get_average_prediction_error()
    
    adjusted_params = base_params.copy()
    
    # Adjust detection sensitivity based on performance
    if quality < 30 or detection_rate < 50:
        # Poor performance - increase sensitivity
        adjusted_params['detection_threshold_factor'] = base_params.get('detection_threshold_factor', ANOMALY_FACTOR) * 0.8
        adjusted_params['max_samples'] = int(base_params.get('max_samples', 50) * 1.2)
        adjusted_params['search_radius_deg'] = base_params.get('search_radius_deg', 5.0) * 1.3
        print("Poor tracking performance detected - increasing sensitivity")
        
    elif quality > 80 and detection_rate > 90 and avg_error < 2.0:
        # Excellent performance - optimize for efficiency
        adjusted_params['detection_threshold_factor'] = base_params.get('detection_threshold_factor', ANOMALY_FACTOR) * 1.2
        adjusted_params['max_samples'] = int(base_params.get('max_samples', 50) * 0.8)
        adjusted_params['search_radius_deg'] = base_params.get('search_radius_deg', 5.0) * 0.7
        print("Excellent tracking performance - optimizing for efficiency")
    
    return adjusted_params

def reset_servo_to_safe_position(pi, target_elevation=None):
    """
    Reset the servo to a safe elevation position.
    
    Args:
        pi: pigpio instance
        target_elevation: Target elevation (if None, uses middle of range)
        
    Returns:
        float: Actual elevation achieved
    """
    from constants import SERVO_SWEEP_START, SERVO_SWEEP_END
    
    if target_elevation is None:
        target_elevation = (SERVO_SWEEP_START + SERVO_SWEEP_END) / 2
    
    # Ensure elevation is within valid range
    safe_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, target_elevation))
    
    print(f"Resetting servo to safe elevation: {safe_elevation:.1f}° (range: {SERVO_SWEEP_START}°-{SERVO_SWEEP_END}°)")
    
    # Move servo to safe position
    from move_motors import set_servo_angle
    set_servo_angle(pi, safe_elevation)
    
    # Wait for servo to settle
    time.sleep(0.5)
    
    print(f"Servo reset complete. Current elevation: {safe_elevation:.1f}°")
    return safe_elevation

def debug_elevation_movement(pi, current_elevation, target_elevation, servo_pin):
    """
    Debug elevation movement and servo status.
    
    Args:
        pi: pigpio instance
        current_elevation: Current elevation position
        target_elevation: Target elevation position
        servo_pin: Servo pin number
    """
    from constants import SERVO_SWEEP_START, SERVO_SWEEP_END
    
    print(f"=== ELEVATION DEBUG ===")
    print(f"Current elevation: {current_elevation:.1f}°")
    print(f"Target elevation: {target_elevation:.1f}°")
    print(f"Servo range: {SERVO_SWEEP_START}° to {SERVO_SWEEP_END}°")
    
    # Check if target is within range
    if target_elevation < SERVO_SWEEP_START or target_elevation > SERVO_SWEEP_END:
        print(f"WARNING: Target elevation {target_elevation:.1f}° is outside servo range!")
        print(f"Valid range is {SERVO_SWEEP_START}° to {SERVO_SWEEP_END}°")
    
    # Check servo pulse width
    try:
        current_pulse = pi.get_servo_pulsewidth(servo_pin)
        print(f"Current servo pulse width: {current_pulse}μs")
        
        # Calculate expected pulse width
        from constants import MIN_PULSE_WIDTH, MAX_PULSE_WIDTH
        expected_pulse = MIN_PULSE_WIDTH + (target_elevation / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
        print(f"Expected pulse width for {target_elevation:.1f}°: {expected_pulse:.0f}μs")
        
        if abs(current_pulse - expected_pulse) > 50:  # 50μs tolerance
            print(f"WARNING: Servo pulse width mismatch! Current: {current_pulse}μs, Expected: {expected_pulse:.0f}μs")
        
    except Exception as e:
        print(f"Error reading servo pulse width: {e}")
    
    print("=== END ELEVATION DEBUG ===")