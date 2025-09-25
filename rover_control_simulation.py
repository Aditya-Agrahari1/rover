from gpiozero import AngularServo
from time import sleep

# GPIO18 with hardware PWM
servo = AngularServo(
    18,
    min_angle=0,
    max_angle=180,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)

while True:
    # Right (90°)
    servo.angle = 90
    sleep(1)

    # Back to center (0°)
    servo.angle = 0
    sleep(1)

    # Left (180°)
    servo.angle = 180
    sleep(1)

    # Back to center (0°)
    servo.angle = 0
    sleep(1)

    # Wait 2 seconds
    sleep(2)
