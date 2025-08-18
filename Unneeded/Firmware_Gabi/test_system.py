#!/usr/bin/env python3
# test_system.py
# Simple test script to verify basic system functionality

import sys
import time

def test_imports():
    """Test that all required modules can be imported."""
    print("Testing module imports...")
    
    try:
        import constants
        print("✓ constants.py imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import constants.py: {e}")
        return False
    
    try:
        import stepper_setup
        print("✓ stepper_setup.py imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import stepper_setup.py: {e}")
        return False
    
    try:
        import move_motors
        print("✓ move_motors.py imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import move_motors.py: {e}")
        return False
    
    try:
        import lidar_reader
        print("✓ lidar_reader.py imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import lidar_reader.py: {e}")
        return False
    
    try:
        import calibration
        print("✓ calibration.py imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import calibration.py: {e}")
        return False
    
    try:
        import scanning
        print("✓ scanning.py imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import scanning.py: {e}")
        return False
    
    try:
        import anomaly_detection
        print("✓ anomaly_detection.py imported successfully")
    except ImportError as e:
        print(f"✗ Failed to import anomaly_detection.py: {e}")
        return False
    
    return True

def test_constants():
    """Test that all required constants are defined."""
    print("\nTesting constants...")
    
    import constants
    
    required_constants = [
        'DIR_PIN', 'STEP_PIN', 'ENABLE_PIN', 'RESET_PIN',
        'M0_PIN', 'M1_PIN', 'M2_PIN', 'SERVO_PIN',
        'LIDAR_SERIAL_PORT', 'LIDAR_BAUD_RATE',
        'STEPS_PER_REVOLUTION', 'STEPPER_PULSE_DELAY',
        'MIN_PULSE_WIDTH', 'MAX_PULSE_WIDTH',
        'SERVO_SWEEP_START', 'SERVO_SWEEP_END',
        'DEFAULT_CALIBRATION_DISTANCE', 'SENSOR_MAX',
        'ANOMALY_FACTOR', 'SWEEP_RANGE'
    ]
    
    for const in required_constants:
        if hasattr(constants, const):
            print(f"✓ {const} = {getattr(constants, const)}")
        else:
            print(f"✗ Missing constant: {const}")
            return False
    
    return True

def test_anomaly_detection():
    """Test anomaly detection logic."""
    print("\nTesting anomaly detection...")
    
    import anomaly_detection
    
    # Test data
    test_calibration = [
        [150.0, 45.0, 10.0],
        [200.0, 47.0, 12.0],
        [180.0, 43.0, 8.0]
    ]
    
    # Test reference distance calculation
    ref_distance = anomaly_detection.get_reference_distance(45.0, 10.0, test_calibration)
    print(f"✓ Reference distance at (45°, 10°): {ref_distance:.1f}cm")
    
    # Test anomaly detection
    is_anomaly = anomaly_detection.is_anomaly(100.0, ref_distance, 0.6)
    print(f"✓ Anomaly detection (100cm vs {ref_distance:.1f}cm): {is_anomaly}")
    
    return True

def main():
    """Run all tests."""
    print("=== Drone Tracking System Test ===")
    print("Testing basic functionality...\n")
    
    tests = [
        test_imports,
        test_constants,
        test_anomaly_detection
    ]
    
    all_passed = True
    for test in tests:
        if not test():
            all_passed = False
    
    print("\n" + "="*40)
    if all_passed:
        print("✓ All tests passed! System is ready.")
        print("\nTo run the system:")
        print("1. Ensure pigpio daemon is running: sudo pigpiod")
        print("2. Connect hardware according to pin definitions")
        print("3. Run: python3 main.py")
    else:
        print("✗ Some tests failed. Please check the errors above.")
        sys.exit(1)

if __name__ == "__main__":
    main()
