from constants import *
from coordinate_transfer import xyz_to_polar
import RPi.GPIO as GPIO  # For direct control of stepper motor GPIO pins
import time

def set_servo_angle(pi, angle):
    """Calculates the required pulse width and sets the servo to a specific angle."""
    pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

#Single step of the stepper motor
def stepper_step():
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(STEPPER_PULSE_DELAY)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(STEPPER_PULSE_DELAY)

#Reset stepper motor position based on steps tracked from beginning
def reset_stepper_pos(stepper_steps_taken):
    state = GPIO.input(DIR_PIN)
    if state:
        GPIO.output(DIR_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    if stepper_steps_taken > 0:
        for _ in range(stepper_steps_taken):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
    GPIO.output(DIR_PIN, GPIO.HIGH)
    print("Stepper pos reset and dir pin set to HIGH")


def move_to_xyz_position(pi, x, y, z, current_azimuth_steps=0, absolute_steps=0):
    """
    Move to an XYZ position while respecting the 360-degree rotation limit.
    
    Args:
        pi: pigpio instance
        x, y, z: Target coordinates in cm
        current_azimuth_steps: Current position in steps (within one revolution)
        absolute_steps: Total steps taken from starting position (tracks multiple revolutions)
    """
    # Convert to polar coordinates
    target_azimuth, target_elevation, distance = xyz_to_polar(x, y, z)
    
    # Check if elevation is within servo limits
    if target_elevation < SERVO_SWEEP_START or target_elevation > SERVO_SWEEP_END:
        print(f"Warning: Target elevation {target_elevation:.1f}° is outside servo range "
              f"({SERVO_SWEEP_START}°-{SERVO_SWEEP_END}°)")
        target_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, target_elevation))
        print(f"Clamped elevation to: {target_elevation:.1f}°")
    
    
    # Calculate stepper movement
    # Convert target azimuth to steps
    target_steps = int((target_azimuth / 360.0) * STEPS_PER_REVOLUTION)
    steps_to_move = target_steps - current_azimuth_steps
    
    # Handle wrap-around for shortest path
    if abs(steps_to_move) > STEPS_PER_REVOLUTION // 2:
        if steps_to_move > 0:
            steps_to_move -= STEPS_PER_REVOLUTION
        else:
            steps_to_move += STEPS_PER_REVOLUTION
            
    # Check if the move would exceed rotation limits
    new_absolute_steps = absolute_steps + steps_to_move
    max_steps = STEPS_PER_REVOLUTION  # One full revolution
    
    if abs(new_absolute_steps) > max_steps:
        print(f"Warning: Movement would exceed ±360° limit. Movement restricted.")
        # Calculate maximum allowed movement in the desired direction
        if steps_to_move > 0:
            steps_to_move = max_steps - absolute_steps
        else:
            steps_to_move = -max_steps - absolute_steps
            
        if steps_to_move == 0:
            print("Cannot move further in this direction. At rotation limit.")
    
    # Move stepper motor
    if steps_to_move != 0:
        # Set direction
        if steps_to_move > 0:
            GPIO.output(DIR_PIN, GPIO.HIGH)  # Clockwise
            direction = "CW"
        else:
            GPIO.output(DIR_PIN, GPIO.LOW)   # Counter-clockwise
            direction = "CCW"
        
        # Execute steps
        for _ in range(abs(steps_to_move)):
            stepper_step()
        
        # Update positions
        new_azimuth_steps = current_azimuth_steps + steps_to_move
        new_absolute_steps = absolute_steps + steps_to_move
        
        # Keep steps in valid range
        new_azimuth_steps = new_azimuth_steps % STEPS_PER_REVOLUTION
    else:
        new_azimuth_steps = current_azimuth_steps
        new_absolute_steps = absolute_steps
            
    return new_azimuth_steps, new_absolute_steps, target_azimuth, target_elevation, distance

def move_to_multiple_xyz_positions(pi, positions, start_azimuth_steps=0, start_absolute_steps=0, dwell_time=1.0):
    """
    Move to multiple XYZ positions in sequence while respecting the 360-degree rotation limit.
    
    Args:
        pi: pigpio instance
        positions: List of (x, y, z) tuples in cm
        start_azimuth_steps: Starting stepper position within one revolution
        start_absolute_steps: Starting absolute position tracking multiple revolutions
        dwell_time: Time to wait at each position in seconds
    
    Returns:
        list: Results for each position [(steps, abs_steps, azimuth, elevation, distance), ...]
    """
    current_steps = start_azimuth_steps
    current_absolute_steps = start_absolute_steps
    results = []
    
    print(f"Moving to {len(positions)} positions...")
    
    for i, (x, y, z) in enumerate(positions, 1):
        print(f"\n--- Position {i}/{len(positions)} ---")
        
        result = move_to_xyz_position(pi, x, y, z, current_steps, current_absolute_steps)
        current_steps = result[0]  # Update current position
        current_absolute_steps = result[1]  # Update absolute position
        results.append(result)
        
        if dwell_time > 0:
            print(f"Dwelling for {dwell_time}s...")
            time.sleep(dwell_time)
    
    print("\nAll positions completed!")
    return results

# Example usage and test function
def test_xyz_movement(pi):
    """
    Test function demonstrating XYZ to polar coordinate movement.
    """
    print("=== XYZ to Polar Movement Test ===\n")
    
    # Test positions (x, y, z) in cm
    test_positions = [
        (1, 0, 0),      # Straight ahead
        (0, 1, 0),      # 90° right
        (0, -1, 0),    # 45° right
        (0, 0, 1),      # Straight up
        (1, 1, 1),     # 45° right, 45° up
        (-100, 0, 0),     # Behind (180°)
        (0, -100, 0),     # Left (270°)
        (50, 50, -30),    # 45° right, down (if servo allows)
    ]
    
    print("Testing coordinate conversions:")
    for x, y, z in test_positions:
        az, el, dist = xyz_to_polar(x, y, z)
        print(f"({x:4.0f}, {y:4.0f}, {z:4.0f}) → Az:{az:6.1f}°, El:{el:5.1f}°, Dist:{dist:6.1f}cm")
    
    print(f"\nServo elevation range: {SERVO_SWEEP_START}° to {SERVO_SWEEP_END}°")
    print("Filtering positions within servo range...")
    
    # Filter positions that are within servo elevation range
    valid_positions = []
    for x, y, z in test_positions:
        _, elevation, _ = xyz_to_polar(x, y, z)
        if SERVO_SWEEP_START <= elevation <= SERVO_SWEEP_END:
            valid_positions.append((x, y, z))
    
    print(f"Found {len(valid_positions)} valid positions")
    
    # Move to valid positions
    if valid_positions:
        results = move_to_multiple_xyz_positions(pi, valid_positions, dwell_time=2.0)
        
        print("\n=== Movement Summary ===")
        for i, ((x, y, z), (steps, abs_steps, az, el, dist)) in enumerate(zip(valid_positions, results)):
            print(f"{i+1}: ({x:4.0f},{y:4.0f},{z:4.0f}) → Az:{az:6.1f}°, El:{el:5.1f}°, Steps:{steps}, Absolute:{abs_steps}")
