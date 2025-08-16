from constants import *
import RPi.GPIO as GPIO
import time

def set_servo_angle(pi, angle):
    """Calculates the required pulse width and sets the servo to a specific angle."""
    pulse_width = MIN_PULSE_WIDTH + (angle / 180.0) * (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

def stepper_step():
    """Execute a single step of the stepper motor."""
    GPIO.output(STEP_PIN, GPIO.HIGH)
    time.sleep(STEPPER_PULSE_DELAY)
    GPIO.output(STEP_PIN, GPIO.LOW)
    time.sleep(STEPPER_PULSE_DELAY)

def reset_stepper_pos(stepper_steps_taken):
    """Reset stepper motor position based on steps tracked from beginning."""
    state = GPIO.input(DIR_PIN)
    if state:
        GPIO.output(DIR_PIN, GPIO.LOW)
    else:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    
    if stepper_steps_taken > 0:
        for _ in range(stepper_steps_taken):
            stepper_step()
    
    GPIO.output(DIR_PIN, GPIO.HIGH)
    stepper_steps_taken = 0
    print("Stepper position reset and direction pin set to HIGH")

def move_to_polar_position(pi, target_azimuth, target_elevation, current_azimuth_steps=0):
    """
    Move motors to a specific polar coordinate position.
    
    Args:
        pi: pigpio instance for servo control
        target_azimuth: Target azimuth angle in degrees
        target_elevation: Target elevation angle in degrees
        current_azimuth_steps: Current stepper motor step count
        
    Returns:
        tuple: (actual_azimuth, target_elevation, new_stepper_steps)
    """
    print(f"Moving to polar coordinates: Az={target_azimuth:.1f}°, El={target_elevation:.1f}°")
    
    # --- ELEVATION CONTROL (SERVO) ---
    # Check if elevation is within servo limits and clamp if necessary
    if target_elevation < SERVO_SWEEP_START or target_elevation > SERVO_SWEEP_END:
        print(f"Warning: Target elevation {target_elevation:.1f}° is outside servo range "
              f"({SERVO_SWEEP_START}°-{SERVO_SWEEP_END}°)")
        target_elevation = max(SERVO_SWEEP_START, min(SERVO_SWEEP_END, target_elevation))
        print(f"Clamped elevation to: {target_elevation:.1f}°")
    
    # Move servo to target elevation
    set_servo_angle(pi, target_elevation)
    print(f"Servo moved to elevation: {target_elevation:.1f}°")
    
    # --- AZIMUTH CONTROL (STEPPER) ---
    # Normalize target azimuth to 0-360 range
    target_azimuth = target_azimuth % 360.0
    
    # Convert target azimuth to steps
    target_steps = int((target_azimuth / 360.0) * STEPS_PER_REVOLUTION)
    
    # Calculate steps needed to move from current position
    steps_to_move = target_steps - current_azimuth_steps
    
    # Handle wrap-around to find shortest path
    if abs(steps_to_move) > STEPS_PER_REVOLUTION // 2:
        if steps_to_move > 0:
            steps_to_move -= STEPS_PER_REVOLUTION
        else:
            steps_to_move += STEPS_PER_REVOLUTION
    
    # Move stepper motor if movement is needed
    if steps_to_move != 0:
        # Set direction
        if steps_to_move > 0:
            GPIO.output(DIR_PIN, GPIO.HIGH)  # Clockwise
            direction = "CW"
        else:
            GPIO.output(DIR_PIN, GPIO.LOW)   # Counter-clockwise
            direction = "CCW"
        
        print(f"Moving stepper {abs(steps_to_move)} steps {direction} "
              f"(from {current_azimuth_steps} to {target_steps})")
        
        # Execute steps
        for step in range(abs(steps_to_move)):
            stepper_step()
            # Optional: Add progress feedback for large movements
            if abs(steps_to_move) > 1000 and step % 500 == 0:
                progress = (step / abs(steps_to_move)) * 100
                print(f"  Progress: {progress:.1f}%")
        
        # Update position
        new_azimuth_steps = current_azimuth_steps + steps_to_move
        
        # Keep steps in valid range (0 to STEPS_PER_REVOLUTION-1)
        new_azimuth_steps = new_azimuth_steps % STEPS_PER_REVOLUTION
        
        # Calculate actual azimuth achieved
        actual_azimuth = (new_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
        
        print(f"Stepper moved to azimuth: {actual_azimuth:.1f}° (steps: {new_azimuth_steps})")
        
    else:
        new_azimuth_steps = current_azimuth_steps
        actual_azimuth = (current_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
        print("No stepper movement needed - already at target azimuth")
    
    print(f"Movement complete: Az={actual_azimuth:.1f}°, El={target_elevation:.1f}°")
    
    return actual_azimuth, target_elevation, new_azimuth_steps