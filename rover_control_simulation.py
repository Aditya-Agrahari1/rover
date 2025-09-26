# =================================================================================
# AgriSense Rover - Main Control Script
# Version: 3.1 (Stable with Clean Shutdown)
# =================================================================================

import requests
import time
import io
from PIL import Image
from gpiozero import Motor, AngularServo
from time import sleep

# --- ⚙️ 1. CONFIGURATION ---

# L298N Motor Driver Pins (BCM numbering)
MOTOR_L_FORWARD = 24
MOTOR_L_BACKWARD = 23
MOTOR_L_ENABLE = 25
MOTOR_R_FORWARD = 17
MOTOR_R_BACKWARD = 27
MOTOR_R_ENABLE = 22

# SG90 Servo Motor Pin
SERVO_PIN = 18

# Network
WEBCAM_URL = "http://192.168.117.249:8080/photo.jpg"   # mobile IP webcam snapshot URL
BACKEND_URL = "https://agrisense-backend-trdc.onrender.com/analyze"

# Rover Behavior
MOTOR_SPEED = 0.75
MOVE_DURATION_S = 3
PAUSE_DURATION_S = 2

# Image Compression
IMAGE_QUALITY = 75  # JPEG quality
MAX_IMAGE_KB = 200  # target compressed size

# Analysis Parameters
LANGUAGE_CODE = "en"
ROW, COL = 1, 1

# --- 🤖 2. HARDWARE INITIALIZATION ---
print("Initializing hardware...")
try:
    motor_left = Motor(forward=MOTOR_L_FORWARD, backward=MOTOR_L_BACKWARD, enable=MOTOR_L_ENABLE)
    motor_right = Motor(forward=MOTOR_R_FORWARD, backward=MOTOR_R_BACKWARD, enable=MOTOR_R_ENABLE)
    servo = AngularServo(SERVO_PIN, min_angle=-90, max_angle=90)
    HARDWARE_OK = True
    print("✅ Motors and Servo initialized successfully.")
except Exception as e:
    print(f"🛑 ERROR: Could not initialize hardware. Details: {e}")
    HARDWARE_OK = False

# --- 🦾 3. CORE FUNCTIONS ---

def move_forward(duration, speed):
    if not HARDWARE_OK: return
    print(f"▶️ Rover moving forward for {duration}s at {int(speed*100)}% speed...")
    motor_left.forward(speed=speed)
    motor_right.forward(speed=speed)
    sleep(duration)

def stop_motors():
    if not HARDWARE_OK: return
    print("⏹️ Rover stopped.")
    motor_left.stop()
    motor_right.stop()

def sweep_servo():
    if not HARDWARE_OK: return
    print("↔️ Performing servo sweep...")
    try:
        servo.angle = 0
        sleep(1)
        servo.angle = -90
        sleep(1)
        servo.angle = 90
        sleep(1)
        servo.angle = 0
        sleep(1)
        print("✅ Sweep complete.")
    except Exception as e:
        print(f"⚠️ Servo sweep failed: {e}")

def resize_image(image_bytes, max_size=(640, 480), max_kb=MAX_IMAGE_KB, quality=IMAGE_QUALITY):
    """Resize + compress image to target size."""
    img = Image.open(io.BytesIO(image_bytes))
    img.thumbnail(max_size)
    while True:
        output = io.BytesIO()
        img.save(output, format="JPEG", quality=quality, optimize=True)
        data = output.getvalue()
        if len(data) <= max_kb * 1024 or quality <= 30:
            return data
        quality -= 5

def capture_and_send():
    print("\n--- Starting Analysis Cycle ---")
    try:
        # Capture image
        print(f"📸 Requesting image from {WEBCAM_URL}...")
        img_response = requests.get(WEBCAM_URL, timeout=10)
        img_response.raise_for_status()
        original_kb = len(img_response.content) / 1024
        print(f"✔️ Image captured ({original_kb:.1f} KB).")

        # Resize + compress
        compressed_bytes = resize_image(img_response.content)
        compressed_kb = len(compressed_bytes) / 1024
        print(f"🗜️ Image compressed to {compressed_kb:.1f} KB.")

        # Upload
        files = {"image": ("snapshot.jpg", compressed_bytes, "image/jpeg")}
        data = {"language_code": LANGUAGE_CODE, "row": ROW, "col": COL}
        print(f"📤 Uploading to {BACKEND_URL}...")
        t_start = time.time()
        backend_response = requests.post(BACKEND_URL, files=files, data=data, timeout=90)
        t_end = time.time()
        backend_response.raise_for_status()
        print(f"✔️ Upload successful! ({backend_response.status_code}) in {t_end - t_start:.2f}s.")

        # Save PDF
        pdf_filename = f"AgriSense_Report_R{ROW}_C{COL}.pdf"
        with open(pdf_filename, "wb") as f:
            f.write(backend_response.content)
        print(f"📄 PDF report saved as '{pdf_filename}'.")

    except requests.exceptions.RequestException as e:
        print(f"🛑 NETWORK ERROR: {e}")
    except Exception as e:
        print(f"🛑 Unexpected error: {e}")

# --- ▶️ 4. MAIN EXECUTION LOOP ---

if __name__ == "__main__":
    if not HARDWARE_OK:
        print("Exiting due to hardware initialization failure.")
    else:
        try:
            servo.angle = 0  # center servo at start
            print("\n--- AgriSense Rover Initialized. Starting main loop (Ctrl+C to stop) ---")

            while True:
                move_forward(MOVE_DURATION_S, MOTOR_SPEED)
                stop_motors()
                sleep(PAUSE_DURATION_S)

                capture_and_send()
                sweep_servo()

                print("\n--- Cycle complete. Waiting before next cycle... ---")
                sleep(5)

        except KeyboardInterrupt:
            print("\n🛑 Program interrupted by user.")
        finally:
            print("🧹 Shutting down... cleaning up GPIO.")
            if HARDWARE_OK:
                motor_left.stop()
                motor_right.stop()
                servo.angle = None   # release PWM safely
            print("✅ Shutdown complete.")
