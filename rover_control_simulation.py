from gpiozero import AngularServo
from time import sleep

# Use GPIO18 (supports hardware PWM)
servo = AngularServo(18, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)

while True:
    # Right 90°
    servo.angle = 90
    sleep(1)

    # Back to center
    servo.angle = 45
    sleep(1)

    # Left -90° (or  -90 if calibrated)
    servo.angle = 90
    sleep(1)

    # Back to center
    servo.angle = 135
    sleep(1)

    # Wait 2 seconds
    sleep(2)
