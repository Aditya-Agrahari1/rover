from gpiozero import Servo
from time import sleep

# Use GPIO pin 18 for the servo's signal wire.
# You can change this number if you use a different pin.
servo = Servo(18)

print("Starting servo control script. Press Ctrl+C to exit.")

try:
    # It's good practice to start in a known position.
    # We'll start at the center.
    print("Initializing at center position (0)...")
    servo.value = 0
    sleep(2)

    # This is the infinite loop for your pattern.
    while True:
        # --- First part of the pattern: Left and back ---
        
        # 1. Rotate 90 degrees LEFT from center
        print("Moving LEFT (-90 degrees)")
        servo.value = -1  # Go to the minimum angle
        
        # 2. Wait for 2 seconds
        sleep(2)
        
        # 3. Go back 90 degrees RIGHT to the original (center) position
        print("Returning to CENTER")
        servo.value = 0   # Go to the middle angle
        
        # 4. Wait for another 2 seconds
        sleep(2)

        # --- Second part of the pattern: Right and back ---

        # 5. Rotate 90 degrees RIGHT from center
        print("Moving RIGHT (+90 degrees)")
        servo.value = 1   # Go to the maximum angle
        
        # 6. Wait for 2 seconds
        sleep(2)

        # 7. Go back 90 degrees LEFT to the original (center) position
        print("Returning to CENTER")
        servo.value = 0   # Go back to the middle angle
        
        # 8. Wait for 2 seconds before repeating the whole loop
        sleep(2)

except KeyboardInterrupt:
    # This block runs when you press Ctrl+C to stop the script.
    print("\nScript stopped by user.")

finally:
    # This ensures the servo is properly shut down and GPIO is cleaned up.
    print("Cleaning up GPIO...")
    servo.close()
