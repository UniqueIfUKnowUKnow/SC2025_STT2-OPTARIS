import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def spherical_to_cartesian(distance, azimuth, elevation):
    """
    Convert spherical coordinates to Cartesian coordinates.
    
    Parameters:
    distance: radial distance from origin
    azimuth: horizontal angle in degrees (0° = North, 90° = East)
    elevation: vertical angle in degrees (0° = horizontal, 90° = straight up)
    
    Returns:
    x, y, z coordinates
    """
    # Convert degrees to radians
    az_rad = np.radians(azimuth)
    el_rad = np.radians(elevation)
    
    # Convert to Cartesian coordinates
    x = distance * np.cos(el_rad) * np.sin(az_rad)
    y = distance * np.cos(el_rad) * np.cos(az_rad)
    z = distance * np.sin(el_rad)
    
    return x, y, z

def plot_3d_points(csv_file):
    """
    Read CSV file and plot points in 3D space.
    
    Parameters:
    csv_file: path to CSV file with columns [distance, azimuth, elevation]
    """
    # Read the CSV file
    # Assumes columns are named 'distance', 'azimuth', 'elevation'
    # If your columns have different names, adjust accordingly
    try:
        df = pd.read_csv(csv_file)
        
        # If columns don't have headers, use positional indexing
        if len(df.columns) == 3 and df.columns[0].startswith('Unnamed'):
            df.columns = ['distance', 'azimuth', 'elevation']
        elif 'distance' not in df.columns:
            # Assume first three columns are distance, azimuth, elevation
            df.columns = ['distance', 'azimuth', 'elevation'] + list(df.columns[3:])
            
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # Extract the measurements
    distances = df['distance'].values
    azimuths = df['azimuth'].values
    elevations = df['elevation'].values
    
    # Convert to Cartesian coordinates
    x_coords, y_coords, z_coords = spherical_to_cartesian(distances, azimuths, elevations)
    
    # Create 3D plot
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the points
    scatter = ax.scatter(x_coords, y_coords, z_coords, 
                        c=distances, cmap='viridis', 
                        s=50, alpha=0.7)
    
    # Add origin point
    ax.scatter([0], [0], [0], color='red', s=100, label='Origin')
    
    # Customize the plot
    ax.set_xlabel('X (East-West)')
    ax.set_ylabel('Y (North-South)')
    ax.set_zlabel('Z (Up-Down)')
    ax.set_title('3D Plot of Distance Measurements')
    
    # Add colorbar for distance
    plt.colorbar(scatter, ax=ax, label='Distance', shrink=0.8)
    
    # Add legend
    ax.legend()
    
    # Make axes equal for better visualization
    max_range = max(np.ptp(x_coords), np.ptp(y_coords), np.ptp(z_coords)) / 2.0
    mid_x = (x_coords.max() + x_coords.min()) * 0.5
    mid_y = (y_coords.max() + y_coords.min()) * 0.5
    mid_z = (z_coords.max() + z_coords.min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    # Show the plot
    plt.tight_layout()
    plt.show()
    
    # Print some statistics
    print(f"Total points plotted: {len(distances)}")
    print(f"Distance range: {distances.min():.2f} to {distances.max():.2f}")
    print(f"Azimuth range: {azimuths.min():.1f}° to {azimuths.max():.1f}°")
    print(f"Elevation range: {elevations.min():.1f}° to {elevations.max():.1f}°")
    
    return x_coords, y_coords, z_coords

def plot_2d_projections(csv_file):
    """
    Create 2D projection plots for additional visualization.
    """
    df = pd.read_csv(csv_file)
    
    if len(df.columns) == 3 and df.columns[0].startswith('Unnamed'):
        df.columns = ['distance', 'azimuth', 'elevation']
    elif 'distance' not in df.columns:
        df.columns = ['distance', 'azimuth', 'elevation'] + list(df.columns[3:])
    
    distances = df['distance'].values
    azimuths = df['azimuth'].values
    elevations = df['elevation'].values
    
    x_coords, y_coords, z_coords = spherical_to_cartesian(distances, azimuths, elevations)
    
    # Create 2D projections
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # XY projection (top view)
    axes[0,0].scatter(x_coords, y_coords, c=distances, cmap='viridis', alpha=0.7)
    axes[0,0].set_xlabel('X (East-West)')
    axes[0,0].set_ylabel('Y (North-South)')
    axes[0,0].set_title('XY Projection (Top View)')
    axes[0,0].grid(True, alpha=0.3)
    axes[0,0].set_aspect('equal')
    
    # XZ projection (side view)
    axes[0,1].scatter(x_coords, z_coords, c=distances, cmap='viridis', alpha=0.7)
    axes[0,1].set_xlabel('X (East-West)')
    axes[0,1].set_ylabel('Z (Up-Down)')
    axes[0,1].set_title('XZ Projection (Side View)')
    axes[0,1].grid(True, alpha=0.3)
    
    # YZ projection (front view)
    axes[1,0].scatter(y_coords, z_coords, c=distances, cmap='viridis', alpha=0.7)
    axes[1,0].set_xlabel('Y (North-South)')
    axes[1,0].set_ylabel('Z (Up-Down)')
    axes[1,0].set_title('YZ Projection (Front View)')
    axes[1,0].grid(True, alpha=0.3)
    
    # Polar plot (azimuth vs elevation)
    axes[1,1].scatter(azimuths, elevations, c=distances, cmap='viridis', alpha=0.7)
    axes[1,1].set_xlabel('Azimuth (degrees)')
    axes[1,1].set_ylabel('Elevation (degrees)')
    axes[1,1].set_title('Azimuth vs Elevation')
    axes[1,1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

# Example usage
if __name__ == "__main__":
    # Replace 'your_file.csv' with the actual path to your CSV file
    csv_filename = 'lidar_calibration_20250815_111410.csv'
    
    print("Plotting 3D points...")
    coords = plot_3d_points(csv_filename)
    
    print("\nCreating 2D projections...")
    plot_2d_projections(csv_filename)