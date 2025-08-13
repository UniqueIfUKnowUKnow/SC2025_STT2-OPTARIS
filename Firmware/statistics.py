# Enhanced distance averaging functions for handling sharp edges

import statistics
import numpy as np
from sklearn.cluster import DBSCAN
from collections import Counter

def cluster_based_averaging(readings, eps=50, min_samples=2):
    """
    Use DBSCAN clustering to identify distinct distance groups,
    then average within the largest cluster.
    
    Args:
        readings: List of distance measurements
        eps: Maximum distance between samples for clustering (cm)
        min_samples: Minimum samples to form a cluster
    
    Returns:
        float: Average distance from the most significant cluster
    """
    if len(readings) <= 2:
        return statistics.mean(readings)
    
    # Reshape for DBSCAN (needs 2D array)
    X = np.array(readings).reshape(-1, 1)
    
    # Apply DBSCAN clustering
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(X)
    labels = clustering.labels_
    
    # Find the largest cluster (excluding noise labeled as -1)
    valid_labels = [label for label in labels if label != -1]
    if not valid_labels:
        # If no clusters found, fall back to median
        return statistics.median(readings)
    
    # Get the most common cluster
    most_common_cluster = Counter(valid_labels).most_common(1)[0][0]
    
    # Average readings from the largest cluster
    cluster_readings = [readings[i] for i, label in enumerate(labels) 
                       if label == most_common_cluster]
    
    return statistics.mean(cluster_readings)

def histogram_based_averaging(readings, bin_width=20):
    """
    Create histogram bins and average within the most populated bin.
    Good for identifying the primary surface while ignoring edge artifacts.
    
    Args:
        readings: List of distance measurements
        bin_width: Width of histogram bins in cm
    
    Returns:
        float: Average distance from the most populated bin
    """
    if len(readings) <= 2:
        return statistics.mean(readings)
    
    min_dist = min(readings)
    max_dist = max(readings)
    
    # Create bins
    bins = []
    current_bin_start = min_dist
    while current_bin_start < max_dist:
        bin_end = current_bin_start + bin_width
        bin_readings = [r for r in readings if current_bin_start <= r < bin_end]
        if bin_readings:
            bins.append((current_bin_start, bin_end, bin_readings))
        current_bin_start = bin_end
    
    # Find the bin with the most readings
    if not bins:
        return statistics.mean(readings)
    
    most_populated_bin = max(bins, key=lambda x: len(x[2]))
    return statistics.mean(most_populated_bin[2])

def median_absolute_deviation_filter(readings, threshold=2.0):
    """
    Use Median Absolute Deviation (MAD) for more robust outlier detection.
    MAD is less sensitive to outliers than standard deviation.
    
    Args:
        readings: List of distance measurements
        threshold: MAD multiplier for outlier threshold
    
    Returns:
        float: Average of filtered readings
    """
    if len(readings) <= 2:
        return statistics.mean(readings)
    
    median = statistics.median(readings)
    mad = statistics.median([abs(r - median) for r in readings])
    
    # If MAD is 0, all values are the same
    if mad == 0:
        return median
    
    # Filter out outliers
    filtered_readings = [r for r in readings 
                        if abs(r - median) <= threshold * mad]
    
    if not filtered_readings:
        return median
    
    return statistics.mean(filtered_readings)

def percentile_trimmed_mean(readings, trim_percent=20):
    """
    Calculate trimmed mean by removing extreme percentiles.
    Helps with edge effects while preserving the main signal.
    
    Args:
        readings: List of distance measurements
        trim_percent: Percentage to trim from each end
    
    Returns:
        float: Trimmed mean of readings
    """
    if len(readings) <= 2:
        return statistics.mean(readings)
    
    sorted_readings = sorted(readings)
    n = len(sorted_readings)
    trim_count = int(n * trim_percent / 200)  # Divide by 200 because we trim both ends
    
    if trim_count >= n // 2:
        return statistics.median(readings)
    
    trimmed = sorted_readings[trim_count:n-trim_count]
    return statistics.mean(trimmed)

def adaptive_averaging(readings, min_cluster_size=3):
    """
    Adaptive approach that tries multiple methods and selects the best one.
    
    Args:
        readings: List of distance measurements
        min_cluster_size: Minimum size for a cluster to be considered valid
    
    Returns:
        tuple: (average_distance, method_used, confidence_score)
    """
    if len(readings) <= 2:
        return statistics.mean(readings), "simple_mean", 1.0
    
    # Try different methods
    methods = {}
    
    # Method 1: Cluster-based (if we have enough data)
    if len(readings) >= min_cluster_size:
        try:
            methods['cluster'] = cluster_based_averaging(readings)
        except:
            pass
    
    # Method 2: Histogram-based
    methods['histogram'] = histogram_based_averaging(readings)
    
    # Method 3: MAD filtering
    methods['mad'] = median_absolute_deviation_filter(readings)
    
    # Method 4: Trimmed mean
    methods['trimmed'] = percentile_trimmed_mean(readings)
    
    # Method 5: Simple median (always reliable)
    methods['median'] = statistics.median(readings)
    
    # Calculate confidence based on consistency between methods
    values = list(methods.values())
    consistency = 1.0 / (1.0 + statistics.stdev(values)) if len(set(values)) > 1 else 1.0
    
    # Choose method based on data characteristics
    range_span = max(readings) - min(readings)
    
    if range_span > 100:  # Large spread suggests multiple surfaces
        if 'cluster' in methods:
            return methods['cluster'], "cluster", consistency
        else:
            return methods['histogram'], "histogram", consistency
    elif range_span > 50:  # Moderate spread
        return methods['mad'], "mad", consistency
    else:  # Small spread
        return methods['trimmed'], "trimmed", consistency

# Modified version of your calibration function with enhanced averaging
def enhanced_calibrate_environment(pi, lidar_data_queue):
    """
    Enhanced calibration with better edge handling for distance averaging.
    """
    print("Starting enhanced calibration with edge detection...")
    
    calibration_data = []
    current_elevation = 0
    set_servo_angle(pi, current_elevation)
    time.sleep(1)
    
    current_physical_azimuth = 0.0
    stepper_direction_cw = True
    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    elevation_positions = list(range(SERVO_SWEEP_START, SERVO_SWEEP_END, 2))
    azimuth_increments = list(range(0, (STEPPER_SWEEP_DEGREES+1), 2))
    stepper_steps_taken = 0
    
    for elevation in elevation_positions:
        print(f"Calibrating at elevation: {elevation}°")
        
        set_servo_angle(pi, elevation)
        time.sleep(0.3)
        
        azimuth_readings = {azimuth: [] for azimuth in azimuth_increments}
        sweep_forward = ((SERVO_SWEEP_START - elevation) % 4 == 0)
        
        # ... (motor movement code remains the same) ...
        
        # Enhanced processing of collected readings
        for azimuth in azimuth_increments:
            readings = azimuth_readings[azimuth]
            if readings and len(readings) >= 3:  # Need minimum readings for analysis
                
                # Use adaptive averaging for better edge handling
                average_distance, method_used, confidence = adaptive_averaging(readings)
                
                # Store additional metadata about the measurement quality
                measurement_data = {
                    'distance': average_distance,
                    'azimuth': azimuth,
                    'elevation': elevation,
                    'raw_count': len(readings),
                    'method': method_used,
                    'confidence': confidence,
                    'range_span': max(readings) - min(readings)
                }
                
                calibration_data.append([average_distance, azimuth, elevation])
                
                print(f"  Az {azimuth}°: {len(readings)} readings, "
                      f"avg = {average_distance:.1f}cm (method: {method_used}, "
                      f"conf: {confidence:.2f}, range: {max(readings)-min(readings):.1f}cm)")
    
    print(f"Enhanced calibration complete! Collected {len(calibration_data)} measurements.")
    return calibration_data