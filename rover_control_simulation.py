# AgriSense Rover Control Script - SIMULATION MODE
# Version: 2.0 (for Raspberry Pi 5)
# Description:
#   Controls rover movement with an L298N motor driver and a servo
#   using gpiozero (works with Pi 5). This is a simulation-friendly
#   script; adjust pins according to your wiring.

from gpiozero import Motor, Servo
from time import sleep

# --- CONFIGURATION ---
# Motor driver pins (BCM numbering)
LEFT_MOTOR_FORWARD = 17
LEFT_MOTOR_BACKWARD = 27
RIGHT_MOTOR_FORWARD = 23
RIGHT_MOTOR_BACKWARD = 24

# Servo pin (BCM numbering)
SERVO_PIN = 18

# --- SETUP ---
# Motor setup
left_motor = Motor(forward=LEFT_MOTOR_FORWARD, backward=LEFT_MOTOR_BACKWARD)
right_motor = Motor(forward=RIGHT_MOTOR_FORWARD, backward=RIGHT_MOTOR_BACKWARD)

# Servo setup
servo = Servo(SERVO_PIN)

# --- MOVEMENT FUNCTIONS ---
def move_forward(duration=2):
    print("Moving forward...")
    left_motor.forward()
    right_motor.forward()
    sleep(duration)
    stop()

def move_backward(duration=2):
    print("Moving backward...")
    left_motor.backward()
    right_motor.backward()
    sleep(duration)
    stop()

def turn_left(duration=1):
    print("Turning left...")
    left_motor.backward()
    right_motor.forward()
    sleep(duration)
    stop()

def turn_right(duration=1):
    print("Turning right...")
    left_motor.forward()
    right_motor.backward()
    sleep(duration)
    stop()

def stop():
    print("Stopping...")
    left_motor.stop()
    right_motor.stop()

def servo_scan():
    """Moves servo right -> center -> left -> center"""
    print("Servo moving right...")
    servo.max()
    sleep(1)

    print("Servo center...")
    servo.mid()
    sleep(1)

    print("Servo moving left...")
    servo.min()
    sleep(1)

    print("Servo back to center...")
    servo.mid()
    sleep(1)

# --- MAIN LOOP ---
try:
    print("Starting rover simulation. Press Ctrl+C to stop.")
    while True:
        move_forward(2)
        turn_left(1)
        move_forward(2)
        turn_right(1)

        servo_scan()

        print("Pausing 2 seconds...")
        sleep(2)

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    print("Cleaning up (motors + servo stopped).")
    stop()
    servo.mid()  # Reset servo to center
