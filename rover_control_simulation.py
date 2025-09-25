from gpiozero import Servo
from time import sleep

servo = Servo(18)

while True:
    # Rotate right
    servo.value = 1   # full speed right
    sleep(0.5)        # adjust time for ~90° movement
    servo.value = 0   # stop
    sleep(0.5)

    # Back to center (simulate by reversing briefly)
    servo.value = -1  # full speed left
    sleep(0.5)        
    servo.value = 0   # stop
    sleep(0.5)

    # Rotate left
    servo.value = -1
    sleep(0.5)
    servo.value = 0
    sleep(0.5)

    # Back to center
    servo.value = 1
    sleep(0.5)
    servo.value = 0
    sleep(0.5)

    # Wait before repeating
    sleep(2)
