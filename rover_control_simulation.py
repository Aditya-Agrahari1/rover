from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import warnings

# Suppress the PWM software fallback warning
warnings.filterwarnings("ignore", category=UserWarning)

# Use pigpio for better PWM control
try:
    from gpiozero import Device
    Device.pin_factory = PiGPIOFactory()
    print("Using pigpio pin factory for better servo control")
except:
    print("pigpio not available, using default pin factory")

# SG90 servo configuration
servo = AngularServo(
    18,
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.5/1000,    # 0.5ms pulse width for -90 degrees  
    max_pulse_width=2.5/1000,    # 2.5ms pulse width for +90 degrees
    frame_width=20.0/1000        # 20ms frame width (50Hz)
)

def smooth_move(target_angle, steps=10, delay=0.05):
    """Move servo smoothly to target angle"""
    current = servo.angle if servo.angle is not None else 0
    if current is None:
        current = 0
    
    step_size = (target_angle - current) / steps
    for i in range(steps):
        servo.angle = current + (step_size * (i + 1))
        sleep(delay)

print("SG90 Servo - 30 degree rotation test")

try:
    # Initialize at center position (0 degrees)
    print("Initializing at center position (0 degrees)...")
    servo.angle = 0
    sleep(2)
    
    # Rotate 30 degrees clockwise
    print("Rotating 30 degrees clockwise...")
    smooth_move(30, steps=15, delay=0.03)
    sleep(1)
    
    print("Movement complete. Servo will hold position.")
    print("Press Ctrl+C to stop and clean up.")
    
    # Keep the servo at 30 degrees
    while True:
        sleep(1)

except KeyboardInterrupt:
    print("\nScript stopped by user.")
    
finally:
    # Clean up GPIO
    print("Cleaning up GPIO...")
    servo.angle = 0  # Return to center before closing
    sleep(0.5)
    servo.close()
    print("Servo control ended.")
