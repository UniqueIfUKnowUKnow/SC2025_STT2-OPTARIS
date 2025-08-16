# tracking_config.py
# Configuration file for enhanced tracking capabilities

# --- Tracking Performance Thresholds ---
TRACKING_QUALITY_THRESHOLDS = {
    'EXCELLENT': 80,    # Quality score above which to optimize for efficiency
    'GOOD': 60,         # Quality score above which to maintain current settings
    'POOR': 30,         # Quality score below which to increase sensitivity
    'CRITICAL': 20      # Quality score below which to use emergency settings
}

DETECTION_RATE_THRESHOLDS = {
    'EXCELLENT': 90,    # Detection rate above which to optimize
    'GOOD': 70,         # Detection rate above which to maintain
    'POOR': 50,         # Detection rate below which to increase sensitivity
    'CRITICAL': 30      # Detection rate below which to use emergency settings
}

# --- Adaptive Parameter Adjustment Factors ---
ADAPTIVE_FACTORS = {
    'INCREASE_SENSITIVITY': {
        'detection_threshold_factor': 0.8,    # More sensitive detection
        'max_samples_multiplier': 1.2,        # Collect more samples
        'search_radius_multiplier': 1.3,      # Expand search area
        'settling_time_multiplier': 1.1       # Wait longer for motors to settle
    },
    'OPTIMIZE_EFFICIENCY': {
        'detection_threshold_factor': 1.2,    # Less sensitive but more efficient
        'max_samples_multiplier': 0.8,        # Collect fewer samples
        'search_radius_multiplier': 0.7,      # Tighten search area
        'settling_time_multiplier': 0.9       # Faster motor settling
    },
    'EMERGENCY_SETTINGS': {
        'detection_threshold_factor': 0.6,    # Very sensitive detection
        'max_samples_multiplier': 1.5,        # Collect many samples
        'search_radius_multiplier': 2.0,      # Large search area
        'settling_time_multiplier': 1.3       # Long settling time
    }
}

# --- Multi-Target Tracking Settings ---
MULTI_TARGET_CONFIG = {
    'MAX_TARGETS': 3,                    # Maximum number of targets to track simultaneously
    'TARGET_TIMEOUT_SECONDS': 30.0,      # Time after which to consider target stale
    'MIN_MEASUREMENTS_FOR_KALMAN': 2,    # Minimum measurements before initializing Kalman filter
    'TARGET_QUALITY_DECAY_RATE': 0.95,   # Factor by which target quality decays per missed detection
    'TARGET_QUALITY_BOOST_RATE': 1.05    # Factor by which target quality improves per successful detection
}

# --- Search Pattern Configuration ---
SEARCH_PATTERNS = {
    'SPIRAL': {
        'name': 'Spiral Outward',
        'positions': [
            (0, 0),                    # Center
            (-0.5, 0), (0.5, 0),      # Close horizontal
            (0, -0.5), (0, 0.5),      # Close vertical
            (-1.0, 0), (1.0, 0),      # Full horizontal
            (0, -1.0), (0, 1.0),      # Full vertical
            (-1.0, -1.0), (1.0, 1.0), # Diagonals
            (1.0, -1.0), (-1.0, 1.0)
        ]
    },
    'GRID': {
        'name': 'Grid Pattern',
        'positions': [
            (0, 0),                    # Center
            (-1.0, 0), (1.0, 0),      # Horizontal line
            (0, -1.0), (0, 1.0),      # Vertical line
            (-1.0, -1.0), (1.0, -1.0), # Bottom corners
            (-1.0, 1.0), (1.0, 1.0)   # Top corners
        ]
    },
    'CROSS': {
        'name': 'Cross Pattern',
        'positions': [
            (0, 0),                    # Center
            (-1.0, 0), (1.0, 0),      # Horizontal
            (0, -1.0), (0, 1.0),      # Vertical
            (-0.5, -0.5), (0.5, -0.5), # Close corners
            (-0.5, 0.5), (0.5, 0.5)
        ]
    }
}

# --- Performance Monitoring Settings ---
PERFORMANCE_MONITORING = {
    'DASHBOARD_UPDATE_INTERVAL': 5,    # Update dashboard every N tracking cycles
    'METRICS_HISTORY_LENGTH': 100,     # Number of historical metrics to keep
    'PERFORMANCE_PLOT_INTERVAL': 20,   # Generate performance plots every N cycles
    'DATA_EXPORT_INTERVAL': 50,        # Export data every N cycles
    'STATE_SAVE_INTERVAL': 10          # Save tracking state every N cycles
}

# --- Visualization Settings ---
VISUALIZATION_CONFIG = {
    'FIGURE_SIZE': (15, 10),           # Size of performance plots
    'DPI': 300,                        # Resolution for saved plots
    'COLOR_SCHEME': {
        'EXCELLENT': '#00FF00',        # Green for excellent performance
        'GOOD': '#00AA00',             # Dark green for good performance
        'POOR': '#FFAA00',             # Orange for poor performance
        'CRITICAL': '#FF0000'          # Red for critical performance
    },
    'PLOT_STYLES': {
        'line_width': 2,
        'marker_size': 6,
        'grid_alpha': 0.3,
        'font_size': 12
    }
}

# --- Emergency Recovery Settings ---
EMERGENCY_RECOVERY = {
    'MAX_CONSECUTIVE_MISSES': 5,       # Maximum consecutive misses before emergency mode
    'EMERGENCY_SEARCH_RADIUS': 10.0,   # Degrees to search in emergency mode
    'EMERGENCY_SAMPLE_COUNT': 100,     # Number of samples to collect in emergency mode
    'RECOVERY_ATTEMPTS': 3,            # Number of recovery attempts before giving up
    'RECOVERY_TIMEOUT': 60.0           # Seconds to wait before recovery attempt
}

# --- Logging and Debug Settings ---
LOGGING_CONFIG = {
    'LOG_LEVEL': 'INFO',               # Logging level (DEBUG, INFO, WARNING, ERROR)
    'LOG_TO_FILE': True,               # Whether to log to file
    'LOG_FILENAME': 'tracking.log',    # Log file name
    'LOG_FORMAT': '%(asctime)s - %(levelname)s - %(message)s',
    'MAX_LOG_SIZE_MB': 10,             # Maximum log file size
    'LOG_BACKUP_COUNT': 3              # Number of backup log files
}

# --- Default Tracking Parameters ---
DEFAULT_TRACKING_PARAMS = {
    'detection_threshold_factor': 0.6,  # Base anomaly detection factor
    'max_samples': 50,                  # Maximum LiDAR samples per detection
    'search_radius_deg': 5.0,           # Base search radius in degrees
    'settling_time': 0.3,               # Motor settling time in seconds
    'max_wait_time': 2.0,               # Maximum time to wait for samples
    'early_detection_threshold': 3,     # Number of anomalies for early detection
    'anomaly_ratio_threshold': 0.3,     # Minimum ratio of anomalies for detection
    'confidence_early_termination': 90  # Confidence threshold for early search termination
}

def get_tracking_config():
    """Get the complete tracking configuration dictionary"""
    return {
        'quality_thresholds': TRACKING_QUALITY_THRESHOLDS,
        'detection_rate_thresholds': DETECTION_RATE_THRESHOLDS,
        'adaptive_factors': ADAPTIVE_FACTORS,
        'multi_target_config': MULTI_TARGET_CONFIG,
        'search_patterns': SEARCH_PATTERNS,
        'performance_monitoring': PERFORMANCE_MONITORING,
        'visualization_config': VISUALIZATION_CONFIG,
        'emergency_recovery': EMERGENCY_RECOVERY,
        'logging_config': LOGGING_CONFIG,
        'default_params': DEFAULT_TRACKING_PARAMS
    }

def validate_tracking_config():
    """Validate that the tracking configuration is consistent and reasonable"""
    config = get_tracking_config()
    errors = []
    warnings = []
    
    # Check for reasonable threshold values
    if config['quality_thresholds']['EXCELLENT'] <= config['quality_thresholds']['GOOD']:
        errors.append("EXCELLENT quality threshold must be greater than GOOD threshold")
    
    if config['quality_thresholds']['GOOD'] <= config['quality_thresholds']['POOR']:
        errors.append("GOOD quality threshold must be greater than POOR threshold")
    
    # Check adaptive factors
    for mode, factors in config['adaptive_factors'].items():
        if factors['detection_threshold_factor'] <= 0:
            errors.append(f"Detection threshold factor for {mode} must be positive")
        if factors['max_samples_multiplier'] <= 0:
            errors.append(f"Max samples multiplier for {mode} must be positive")
    
    # Check multi-target config
    if config['multi_target_config']['MAX_TARGETS'] <= 0:
        errors.append("Maximum targets must be positive")
    
    if config['multi_target_config']['TARGET_TIMEOUT_SECONDS'] <= 0:
        errors.append("Target timeout must be positive")
    
    # Check performance monitoring
    if config['performance_monitoring']['DASHBOARD_UPDATE_INTERVAL'] <= 0:
        errors.append("Dashboard update interval must be positive")
    
    # Check default params
    if config['default_params']['detection_threshold_factor'] <= 0:
        errors.append("Default detection threshold factor must be positive")
    
    if config['default_params']['max_samples'] <= 0:
        errors.append("Default max samples must be positive")
    
    # Return validation results
    if errors:
        print("Configuration validation errors:")
        for error in errors:
            print(f"  ERROR: {error}")
        return False
    
    if warnings:
        print("Configuration validation warnings:")
        for warning in warnings:
            print(f"  WARNING: {warning}")
    
    print("Tracking configuration validation passed!")
    return True

if __name__ == "__main__":
    # Validate configuration when run directly
    validate_tracking_config()
    
    # Print configuration summary
    config = get_tracking_config()
    print(f"\nTracking Configuration Summary:")
    print(f"  Quality thresholds: {config['quality_thresholds']}")
    print(f"  Max targets: {config['multi_target_config']['MAX_TARGETS']}")
    print(f"  Default search radius: {config['default_params']['search_radius_deg']}°")
    print(f"  Dashboard updates: every {config['performance_monitoring']['DASHBOARD_UPDATE_INTERVAL']} cycles")
