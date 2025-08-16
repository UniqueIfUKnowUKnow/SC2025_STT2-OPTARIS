import numpy as np
import matplotlib.pyplot as plt
from kalman_filter import DroneTrajectoryKalman

# Example initial data (5 measurements)
# Format: [distance_m, azimuth_degrees, elevation_degrees]
initial_measurements = [
    [1.0, 1.949646, 1.125267],
    [1.0, 2.322887, 1.340508],
    [1.0, 6.786733, 3.903117],
    [1.0, 17.574619, 9.888968],
    [1.0, 21.766408, 12.084309],
]
initial_timestamps = [0, 1, 2, 3, 4]  # seconds

# Example secondary data (new measurements from sensor)
# These would normally come from your sensor after predictions
secondary_measurements = [
    [1.0, 34.056937, 17.917351],
    [1.0, 53.711857, 24.956027],
    [1.0, 57.274385, 25.906169],
    [1.0, 63.101275, 27.243276],
    [1.0, 78.830279, 29.527819],
]

secondary_timestamps = [5, 6, 7, 8]

def main():
    # Initialize Kalman filter
    kf = DroneTrajectoryKalman(measurement_error_deg=1.0)
    
    # Convert degrees to radians for initial data
    initial_rad = []
    for dist, az_deg, el_deg in initial_measurements:
        initial_rad.append([dist, np.radians(az_deg), np.radians(el_deg)])
    
    # Process initial measurements
    print("Processing initial measurements...")
    kf.process_measurement_sequence(initial_rad, initial_timestamps)
    
    # Get predictions for future times
    future_times = [5, 6, 7, 8, 9, 10]
    predictions = kf.predict_future_positions(future_times)
    
    print("Predicted positions:")
    for t, pred in zip(future_times, predictions):
        az_deg = np.degrees(pred[1])
        el_deg = np.degrees(pred[2])
        print(f"t={t}s: dist={pred[0]:.1f}m, az={az_deg:.1f}°, el={el_deg:.1f}°")
    
    # Process secondary measurements (simulate getting new sensor data)
    print("\nProcessing secondary measurements...")
    all_measurements = initial_rad.copy()
    all_timestamps = initial_timestamps.copy()
    
    for i, (measurement, timestamp) in enumerate(zip(secondary_measurements, secondary_timestamps)):
        # Convert to radians
        meas_rad = [measurement[0], np.radians(measurement[1]), np.radians(measurement[2])]
        
        # Predict to this time
        dt = timestamp - all_timestamps[-1]
        kf.predict(dt)
        
        # Update with measurement
        kf.update(meas_rad)
        
        # Store for plotting
        all_measurements.append(meas_rad)
        all_timestamps.append(timestamp)
        
        print(f"Updated with measurement at t={timestamp}s")
    
    # Get final predictions
    final_future_times = [9, 10, 11, 12]
    final_predictions = kf.predict_future_positions(final_future_times)
    
    # Plot results
    plot_trajectory(all_measurements, all_timestamps, predictions, future_times, 
                   final_predictions, final_future_times, len(initial_measurements))

def plot_trajectory(measurements, timestamps, initial_predictions, initial_pred_times,
                   final_predictions, final_pred_times, initial_count):
    """Plot the complete trajectory"""
    
    # Convert to Cartesian coordinates for plotting
    def polar_to_cartesian(polar_coords):
        distance, azimuth, elevation = polar_coords
        x = distance * np.cos(elevation) * np.cos(azimuth)
        y = distance * np.cos(elevation) * np.sin(azimuth)
        z = distance * np.sin(elevation)
        return [x, y, z]
    
    # Convert all measurements
    cartesian_meas = [polar_to_cartesian(m) for m in measurements]
    x_meas = [c[0] for c in cartesian_meas]
    y_meas = [c[1] for c in cartesian_meas]
    z_meas = [c[2] for c in cartesian_meas]
    
    # Convert predictions
    cartesian_init_pred = [polar_to_cartesian(p) for p in initial_predictions]
    x_init_pred = [c[0] for c in cartesian_init_pred]
    y_init_pred = [c[1] for c in cartesian_init_pred]
    z_init_pred = [c[2] for c in cartesian_init_pred]
    
    cartesian_final_pred = [polar_to_cartesian(p) for p in final_predictions]
    x_final_pred = [c[0] for c in cartesian_final_pred]
    y_final_pred = [c[1] for c in cartesian_final_pred]
    z_final_pred = [c[2] for c in cartesian_final_pred]
    
    # Create 3D plot
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot initial measurements (blue)
    ax.scatter(x_meas[:initial_count], y_meas[:initial_count], z_meas[:initial_count], 
              c='blue', s=60, label='Initial Measurements', alpha=0.8)
    
    # Plot secondary measurements (red)
    if len(x_meas) > initial_count:
        ax.scatter(x_meas[initial_count:], y_meas[initial_count:], z_meas[initial_count:], 
                  c='red', s=60, label='Secondary Measurements', alpha=0.8)
    
    # Plot trajectory line
    ax.plot(x_meas, y_meas, z_meas, 'k--', alpha=0.5, linewidth=1)
    
    # Plot initial predictions (green)
    ax.scatter(x_init_pred, y_init_pred, z_init_pred, 
              c='green', s=40, label='Initial Predictions', alpha=0.7, marker='^')
    
    # Plot final predictions (orange)
    ax.scatter(x_final_pred, y_final_pred, z_final_pred, 
              c='orange', s=50, label='Final Predictions', alpha=0.8, marker='s')
    ax.plot(x_final_pred, y_final_pred, z_final_pred, 'orange', linewidth=2, alpha=0.7)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    ax.set_title('Drone Trajectory Tracking Results')
    
    # Add grid
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Also create a 2D plot showing the trajectory progression over time
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
    
    # Distance vs time
    ax1.plot(timestamps[:initial_count], [m[0] for m in measurements[:initial_count]], 'bo-', label='Initial')
    if len(timestamps) > initial_count:
        ax1.plot(timestamps[initial_count:], [m[0] for m in measurements[initial_count:]], 'ro-', label='Secondary')
    ax1.plot(final_pred_times, [p[0] for p in final_predictions], 'o--', color='orange', label='Predictions')
    ax1.set_ylabel('Distance (m)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Azimuth vs time
    ax2.plot(timestamps[:initial_count], [np.degrees(m[1]) for m in measurements[:initial_count]], 'bo-', label='Initial')
    if len(timestamps) > initial_count:
        ax2.plot(timestamps[initial_count:], [np.degrees(m[1]) for m in measurements[initial_count:]], 'ro-', label='Secondary')
    ax2.plot(final_pred_times, [np.degrees(p[1]) for p in final_predictions], 'o--', color='orange', label='Predictions')
    ax2.set_ylabel('Azimuth (°)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Elevation vs time
    ax3.plot(timestamps[:initial_count], [np.degrees(m[2]) for m in measurements[:initial_count]], 'bo-', label='Initial')
    if len(timestamps) > initial_count:
        ax3.plot(timestamps[initial_count:], [np.degrees(m[2]) for m in measurements[initial_count:]], 'ro-', label='Secondary')
    ax3.plot(final_pred_times, [np.degrees(p[2]) for p in final_predictions], 'o--', color='orange', label='Predictions')
    ax3.set_ylabel('Elevation (°)')
    ax3.set_xlabel('Time (s)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()