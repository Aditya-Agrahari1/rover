from gpiozero import Servo
from time import sleep

# Use GPIO18 (hardware PWM pin)
servo = Servo(18)

while True:
    # Right (+90° approx.)
    servo.max()
    sleep(1)

    # Back to center
    servo.mid()
    sleep(1)

    # Left (-90° approx.)
    servo.min()
    sleep(1)

    # Back to center
    servo.mid()
    sleep(1)

    # Wait 2 seconds before repeating
    sleep(2)
