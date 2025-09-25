# Simple Servo Control Script for Raspberry Pi
# Description: This script controls a single servo motor, making it scan
# right, then left, and return to center in a continuous loop.

import RPi.GPIO as GPIO
import time

# --- CONFIGURATION ---
# IMPORTANT: Connect the servo's signal wire to this BCM pin number.
SERVO_PIN = 18 

# --- SETUP ---
# Use BCM pin numbering
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set up the servo pin as a PWM output
GPIO.setup(SERVO_PIN, GPIO.OUT)
# Initialize PWM on the servo pin at 50Hz (standard for servos)
servo = GPIO.PWM(SERVO_PIN, 50) 
servo.start(7.5) # Start the servo at 90 degrees (center)
time.sleep(1)    # Wait for the servo to get to the initial position

def set_angle(angle):
    """Moves the servo to a specific angle (0 to 180 degrees)."""
    # Formula to convert angle to the required PWM duty cycle
    duty_cycle = (angle / 18) + 2.5
    servo.ChangeDutyCycle(duty_cycle)
    # Wait for 1 second to allow the servo to complete its movement
    time.sleep(1)

# --- MAIN LOOP ---
try:
    print("Starting servo scan loop. Press Ctrl+C to stop.")
    while True:
        # 1. Move 90 degrees right (position 180)
        print("Moving right...")
        set_angle(180)

        # 2. Move back to the original position (center)
        print("Returning to center...")
        set_angle(90)

        # 3. Move 90 degrees left (position 0)
        print("Moving left...")
        set_angle(0)

        # 4. Move back to the original position (center)
        print("Returning to center...")
        set_angle(90)

        # 5. Wait for 2 seconds before repeating
        print("Waiting for 2 seconds...")
        time.sleep(2)

except KeyboardInterrupt:
    # This block runs when you press Ctrl+C
    print("\nProgram stopped by user.")

finally:
    # This block runs at the end, no matter what
    print("Cleaning up GPIO...")
    servo.stop()       # Stop the PWM signal
    GPIO.cleanup()     # Reset the GPIO pins
