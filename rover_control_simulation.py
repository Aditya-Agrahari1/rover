from gpiozero import Servo
from time import sleep

# Configure servo for standard 180-degree servo (not continuous rotation)
# Set min_pulse_width and max_pulse_width for standard servo
servo = Servo(18, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)

print("Starting servo control script. Press Ctrl+C to exit.")

try:
    # Initialize at center position (90 degrees)
    print("Initializing at center position (0)...")
    servo.value = 0  # Center position
    sleep(2)
    
    # Main control loop
    while True:
        # 1. Move 90 degrees RIGHT from center
        print("Moving RIGHT (+90 degrees)")
        servo.value = 1  # Maximum position (90 degrees right from center)
        sleep(2)  # Wait 2 seconds
        
        # 2. Return to center (original position)
        print("Returning to CENTER")
        servo.value = 0  # Center position
        sleep(2)  # Wait 2 seconds
        
        # 3. Move 90 degrees LEFT from center
        print("Moving LEFT (-90 degrees)")
        servo.value = -1  # Minimum position (90 degrees left from center)
        sleep(2)  # Wait 2 seconds
        
        # 4. Return to center (original position)
        print("Returning to CENTER")
        servo.value = 0  # Center position
        sleep(2)  # Wait 2 seconds (as requested)
        
        print("Waiting 2 seconds before next cycle...")
        # The cycle will repeat automatically

except KeyboardInterrupt:
    print("\nScript stopped by user.")
    
finally:
    # Clean up GPIO
    print("Cleaning up GPIO...")
    servo.value = 0  # Return to center before closing
    sleep(0.5)
    servo.close()
