from constants import *
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
    
    # Add delay after direction change
    time.sleep(0.001)
    
    if stepper_steps_taken > 0:
        for _ in range(stepper_steps_taken):
            GPIO.output(STEP_PIN, GPIO.HIGH)
            time.sleep(STEPPER_PULSE_DELAY)
            GPIO.output(STEP_PIN, GPIO.LOW)
            time.sleep(STEPPER_PULSE_DELAY)
    
    # Reset to known direction state
    GPIO.output(DIR_PIN, GPIO.HIGH)
    time.sleep(0.001)  # Allow direction to settle
    stepper_steps_taken = 0
    print("Stepper pos reset and dir pin set to HIGH")

def move_to_polar_position(pi, target_azimuth, target_elevation, current_azimuth_steps=0):

    #print(f"Moving to polar coordinates: Az={target_azimuth:.1f}°, El={target_elevation:.1f}°")
    
    # --- ELEVATION CONTROL (SERVO) ---
    # Check if elevation is within servo limits and clamp if necessary
    if target_elevation < 0 or target_elevation > 180:
        # print(f"Warning: Target elevation {target_elevation:.1f}° is outside servo range "
        #       f"({0}°-{180}°)")
        target_elevation = max(0, min(180, target_elevation))
        # print(f"Clamped elevation to: {target_elevation:.1f}°")
    
    # Move servo to target elevation
    set_servo_angle(pi, target_elevation)
    # print(f"Servo moved to elevation: {target_elevation:.1f}°")
    
    # --- AZIMUTH CONTROL (STEPPER) ---
    # Normalize target azimuth to 0-360 range
    target_azimuth = target_azimuth % 360.0
    
    # Convert current steps to current azimuth for debugging
    current_azimuth = (current_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
    # print(f"Current position: Az={current_azimuth:.1f}° (steps: {current_azimuth_steps})")
    
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
    
    # print(f"Steps calculation: target_steps={target_steps}, current_steps={current_azimuth_steps}, steps_to_move={steps_to_move}")
    
    # Move stepper motor if movement is needed
    if steps_to_move != 0:
        # Set direction
        if steps_to_move > 0:
            GPIO.output(DIR_PIN, GPIO.LOW)  # Clockwise
            direction = "CW"
            # print(f"Setting direction: CLOCKWISE (HIGH)")
        else:
            GPIO.output(DIR_PIN, GPIO.HIGH)   # Counter-clockwise
            direction = "CCW"
            # print(f"Setting direction: COUNTER-CLOCKWISE (LOW)")
        
        # CRITICAL: Add delay after direction change to ensure it takes effect
        time.sleep(0.002)  # Increased delay for DRV8825 setup time
        
        # print(f"Moving stepper {abs(steps_to_move)} steps {direction} "
        #       f"(from {current_azimuth_steps} to target {target_steps})")
        
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
        
        # print(f"Stepper moved to azimuth: {actual_azimuth:.1f}° (steps: {new_azimuth_steps})")
        
    else:
        new_azimuth_steps = current_azimuth_steps
        actual_azimuth = (current_azimuth_steps / STEPS_PER_REVOLUTION) * 360.0
        # print("No stepper movement needed - already at target azimuth")
    
    # print(f"Movement complete: Az={actual_azimuth:.1f}°, El={target_elevation:.1f}°")
    
    return actual_azimuth, target_elevation, new_azimuth_steps