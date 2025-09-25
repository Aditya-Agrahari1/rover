from gpiozero import AngularServo
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import warnings

# Suppress the PWM software fallback warning
warnings.filterwarnings("ignore", category=UserWarning)

# Use pigpio for better PWM control (reduces jitter and improves accuracy)
try:
    from gpiozero import Device
    Device.pin_factory = PiGPIOFactory()
    print("Using pigpio pin factory for better servo control")
except:
    print("pigpio not available, using default pin factory")
    print("Install pigpio with: sudo apt install pigpio python3-pigpio")
    print("Then run: sudo systemctl enable pigpiod && sudo systemctl start pigpiod")

# SG90 servo specifications:
# - Operating range: typically 0-180 degrees
# - Pulse width range: 1ms to 2ms (some SG90s use 0.5ms to 2.5ms)
# - Control signal: 20ms period (50Hz)

# Try these pulse width settings for SG90
servo = AngularServo(
    18,
    min_angle=-90,
    max_angle=90,
    min_pulse_width=0.5/1000,    # 0.5ms pulse width for -90 degrees  
    max_pulse_width=2.5/1000,    # 2.5ms pulse width for +90 degrees
    frame_width=20.0/1000        # 20ms frame width (50Hz)
)

print("Starting SG90 servo control script. Press Ctrl+C to exit.")

def smooth_move(target_angle, steps=10, delay=0.05):
    """Move servo smoothly to target angle"""
    current = servo.angle if servo.angle is not None else 0
    if current is None:
        current = 0
    
    step_size = (target_angle - current) / steps
    for i in range(steps):
        servo.angle = current + (step_size * (i + 1))
        sleep(delay)

try:
    # Initialize at center position
    print("Initializing at center position (0 degrees)...")
    servo.angle = 0
    sleep(2)
    
    # Main control loop
    while True:
        # 1. Move 90 degrees RIGHT from center
        print("Moving RIGHT (+90 degrees)")
        smooth_move(90, steps=15, delay=0.03)  # Slower, smoother movement
        sleep(2)  # Wait 2 seconds
        
        # 2. Return to center (original position)
        print("Returning to CENTER")
        smooth_move(0, steps=15, delay=0.03)   # Slower, smoother movement
        sleep(2)  # Wait 2 seconds
        
        # 3. Move 90 degrees LEFT from center
        print("Moving LEFT (-90 degrees)")
        smooth_move(-90, steps=15, delay=0.03) # Slower, smoother movement
        sleep(2)  # Wait 2 seconds
        
        # 4. Return to center (original position)
        print("Returning to CENTER")
        smooth_move(0, steps=15, delay=0.03)   # Slower, smoother movement
        sleep(2)  # Wait 2 seconds
        
        print("Waiting 2 seconds before next cycle...")

except KeyboardInterrupt:
    print("\nScript stopped by user.")
    
finally:
    # Clean up GPIO
    print("Cleaning up GPIO...")
    servo.angle = 0  # Return to center before closing
    sleep(0.5)
    servo.close()

# Alternative pulse width settings to try if the above doesn't work:
# For some SG90 servos, try these settings:
#     min_pulse_width=0.5/1000,    # 0.5ms
#     max_pulse_width=2.5/1000,    # 2.5ms
