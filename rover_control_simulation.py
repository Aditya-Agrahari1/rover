# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from gpiozero import AngularServo
from time import sleep
import requests, time, threading, queue, io
from PIL import Image   # Pillow for image resize

# =========================
# Motor driver pins (BCM numbering)
IN1, IN2, ENA = 24, 23, 25
IN3, IN4, ENB = 17, 27, 22

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)

pwmA = GPIO.PWM(ENA, 100)  # 100 Hz PWM
pwmB = GPIO.PWM(ENB, 100)
pwmA.start(0)
pwmB.start(0)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)

def forward(t=4, speed=70):
    """Move rover in reversed forward direction."""
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    sleep(t)
    stop()

# =========================
# Servo setup (SG90 on GPIO18 – hardware PWM capable)
servo = AngularServo(
    18,
    min_angle=0,
    max_angle=180,
    min_pulse_width=0.0005,
    max_pulse_width=0.0025
)

def sweep_servo():
    """Sweep servo slowly: center → right → center → left → center."""
    for angle in range(0, 91, 2):  
        servo.angle = angle
        sleep(0.05)
    for angle in range(90, -1, -2):  
        servo.angle = angle
        sleep(0.05)
    for angle in range(0, 91, 2):  
        servo.angle = 180 - angle
        sleep(0.05)
    for angle in range(90, 181, 2):  
        servo.angle = angle
        sleep(0.05)

# =========================
# Camera + Backend setup
WEBCAM_URL = "http://192.168.117.249:8080/photo.jpg"   # 📱 Mobile IP Webcam snapshot
BACKEND_URL = "https://rover-backend.onrender.com/analyze"  # 🌐 FastAPI backend
LANGUAGE_CODE = "en"
ROW, COL = 1, 1

# =========================
# Queue + Background Worker
image_queue = queue.Queue()

def resize_image(image_bytes, max_size=(640, 480), max_kb=200):
    """Resize + compress image until under max_kb."""
    img = Image.open(io.BytesIO(image_bytes))
    img.thumbnail(max_size)   # shrink keeping aspect ratio
    quality = 85
    while True:
        output = io.BytesIO()
        img.save(output, format="JPEG", quality=quality, optimize=True)
        data = output.getvalue()
        if len(data) <= max_kb * 1024 or quality <= 30:
            return data
        quality -= 5  # lower quality step by step if still too big

def uploader_worker():
    """Background thread: uploads images from queue."""
    while True:
        snapshot_bytes, idx = image_queue.get()
        try:
            t0 = time.time()
            print(f"[UPLOAD] ⬆️ Uploading image {idx} ({len(snapshot_bytes)/1024:.1f} KB)...")

            files = {"image": (f"snapshot_{idx}.jpg", snapshot_bytes, "image/jpeg")}
            data = {"language_code": LANGUAGE_CODE, "row": ROW, "col": COL}

            response = requests.post(BACKEND_URL, files=files, data=data, timeout=300)

            if response.status_code == 200:
                pdf_filename = f"AgriSense_Report_{ROW}_{COL}_{idx}.pdf"
                with open(pdf_filename, "wb") as f:
                    f.write(response.content)
                print(f"[UPLOAD] ✅ Report saved {pdf_filename} (took {time.time()-t0:.1f}s)")
            else:
                print(f"[UPLOAD] ❌ Backend error {response.status_code}: {response.text}")
        except Exception as e:
            print(f"[UPLOAD] ❌ Failed image {idx}: {e}")
        finally:
            image_queue.task_done()

# Start background uploader
threading.Thread(target=uploader_worker, daemon=True).start()

# =========================
# Capture function
capture_count = 0
def capture_image():
    global capture_count
    try:
        print("[CAPTURE] 📸 Requesting image from webcam...")
        img_response = requests.get(WEBCAM_URL, timeout=10)
        img_response.raise_for_status()

        resized_bytes = resize_image(img_response.content)
        capture_count += 1
        image_queue.put((resized_bytes, capture_count))

        print(f"[CAPTURE] ✅ Image {capture_count} resized "
              f"({len(resized_bytes)/1024:.1f} KB) → queued for upload")
    except Exception as e:
        print(f"[CAPTURE] ❌ Failed: {e}")

# =========================
# Main loop
try:
    while True:
        print("\n[INFO] 🚙 Rover moving forward...")
        forward(2, 70)
        sleep(2)
        print("[INFO] 🛑 Rover stopped.")

        # Capture snapshot and queue upload
        capture_image()

        print("[INFO] 🔄 Servo sweeping...")
        sweep_servo()

except KeyboardInterrupt:
    pass

finally:
    stop()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
    print("[INFO] ✅ GPIO cleanup done.")
