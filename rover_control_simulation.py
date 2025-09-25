# AgriSense Rover Control Script - HARDWARE MODE
# Version: 2.0
# Description: This script controls the physical AgriSense rover.
# It connects to an IP camera, captures images, sends them to a backend,
# and uses the RPi.GPIO library to control motors via an L298N driver.

import RPi.GPIO as GPIO
import time
import requests
import cv2
import numpy as np

# -----------------------------------------------------------------------------
# --- CONFIGURATION & CALIBRATION ---
# --- You MUST adjust these values for your specific rover ---
# -----------------------------------------------------------------------------

# -- Backend Server URL --
BACKEND_URL = "http://YOUR_SERVER_IP:8000/analyze"

# -- IP Camera URL --
IP_CAMERA_URL = "http://YOUR_IP_CAMERA_URL/video"

# -- GPIO Pin Configuration (using BCM numbering) --
# Connect L298N Motor Driver to these pins
MOTOR_L_IN1 = 24
MOTOR_L_IN2 = 23
MOTOR_L_ENA = 25
MOTOR_R_IN3 = 17
MOTOR_R_IN4 = 27
MOTOR_R_ENB = 22

# -- Farm Dimensions --
ROW_LENGTH_M = 10.0
CROPS_PER_ROW = 5
TOTAL_COLUMNS = 3

# -- Rover Calibration --
# !!! CRITICAL: YOU MUST CALIBRATE THESE VALUES BY TESTING YOUR ROVER !!!
MOTOR_SPEED_MPS = 0.2    # Measure this: how many meters your rover travels in 1 second.
NINETY_DEGREE_TURN_TIME_S = 1.5 # Measure this: time needed for a perfect 90-degree turn.
COLUMN_SHIFT_DISTANCE_M = 0.5 # The distance between your crop columns.

# -----------------------------------------------------------------------------
# --- SCRIPT INITIALIZATION ---
# -----------------------------------------------------------------------------

if CROPS_PER_ROW > 1:
    DISTANCE_PER_CROP_M = ROW_LENGTH_M / (CROPS_PER_ROW - 1)
else:
    DISTANCE_PER_CROP_M = 0

# Global PWM objects for speed control
pwm_l = None
pwm_r = None

# --- HARDWARE FUNCTIONS ---

def setup_gpio():
    """Initializes GPIO pins for motor control."""
    global pwm_l, pwm_r
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    motor_pins = [MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_ENA, MOTOR_R_IN3, MOTOR_R_IN4, MOTOR_R_ENB]
    for pin in motor_pins:
        GPIO.setup(pin, GPIO.OUT)
    pwm_l = GPIO.PWM(MOTOR_L_ENA, 100)
    pwm_r = GPIO.PWM(MOTOR_R_ENB, 100)
    pwm_l.start(75) # Set motor speed (0-100)
    pwm_r.start(75) # Set motor speed (0-100)
    print("GPIO setup complete.")

def stop_motors():
    GPIO.output(MOTOR_L_IN1, GPIO.LOW)
    GPIO.output(MOTOR_L_IN2, GPIO.LOW)
    GPIO.output(MOTOR_R_IN3, GPIO.LOW)
    GPIO.output(MOTOR_R_IN4, GPIO.LOW)

def move_forward(duration_s):
    print(f"Moving forward for {duration_s:.2f} seconds...")
    GPIO.output(MOTOR_L_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_L_IN2, GPIO.LOW)
    GPIO.output(MOTOR_R_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_R_IN4, GPIO.LOW)
    time.sleep(duration_s)
    stop_motors()

def turn_right(duration_s):
    print(f"Turning right for {duration_s:.2f} seconds...")
    GPIO.output(MOTOR_L_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_L_IN2, GPIO.LOW)
    GPIO.output(MOTOR_R_IN3, GPIO.LOW)
    GPIO.output(MOTOR_R_IN4, GPIO.HIGH)
    time.sleep(duration_s)
    stop_motors()

def turn_left(duration_s):
    print(f"Turning left for {duration_s:.2f} seconds...")
    GPIO.output(MOTOR_L_IN1, GPIO.LOW)
    GPIO.output(MOTOR_L_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_R_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_R_IN4, GPIO.LOW)
    time.sleep(duration_s)
    stop_motors()

# -----------------------------------------------------------------------------
# --- CORE LOGIC FUNCTIONS ---
# -----------------------------------------------------------------------------

def move_forward_one_plant():
    if MOTOR_SPEED_MPS == 0:
        print("Error: Motor speed is not calibrated (0). Cannot move.")
        return
    move_duration = DISTANCE_PER_CROP_M / MOTOR_SPEED_MPS
    move_forward(move_duration)

def shift_to_next_column():
    print("--- Shifting to the next column ---")
    shift_duration = COLUMN_SHIFT_DISTANCE_M / MOTOR_SPEED_MPS
    turn_right(NINETY_DEGREE_TURN_TIME_S)
    move_forward(shift_duration)
    turn_left(NINETY_DEGREE_TURN_TIME_S)
    print("--- Aligned with the next column ---")

def analyze_plant_at(video_capture, row, col):
    # This function remains the same as in the simulation
    print(f"-> Analyzing plant at (Col: {col}, Row: {row}).")
    print("   Capturing frame from IP camera...")
    ret, frame = video_capture.read()
    if not ret:
        print("   [ERROR] Could not read frame from video stream.")
        return
    is_success, buffer = cv2.imencode(".jpg", frame)
    if not is_success:
        print("   [ERROR] Could not encode frame to JPEG.")
        return
    image_bytes = buffer.tobytes()
    payload = {'language_code': 'en', 'row': row, 'col': col}
    files = {'image': ('image.jpg', image_bytes, 'image/jpeg')}
    try:
        print("   Sending image to backend server...")
        response = requests.post(BACKEND_URL, data=payload, files=files, timeout=90)
        response.raise_for_status()
        print(f"   Success! Backend responded with status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"   [ERROR] Failed to send image to backend: {e}")

# -----------------------------------------------------------------------------
# --- MAIN EXECUTION ---
# -----------------------------------------------------------------------------

def start_farm_scan():
    print("--- Starting AgriSense Farm Scan (HARDWARE MODE) ---")
    cap = cv2.VideoCapture(IP_CAMERA_URL)
    if not cap.isOpened():
        print(f"[FATAL ERROR] Could not open video stream at {IP_CAMERA_URL}")
        return
    print(f"Successfully connected to IP Camera at {IP_CAMERA_URL}")
    time.sleep(2)
    for col in range(TOTAL_COLUMNS):
        print(f"\n===== SCANNING COLUMN {col} =====")
        for row in range(CROPS_PER_ROW):
            analyze_plant_at(cap, row, col)
            if row < CROPS_PER_ROW - 1:
                move_forward_one_plant()
                time.sleep(1)
        if col < TOTAL_COLUMNS - 1:
            shift_to_next_column()
            time.sleep(2)
    cap.release()
    print("\n===== Farm Scan Complete =====")

if __name__ == "__main__":
    try:
        setup_gpio()
        start_farm_scan()
    except KeyboardInterrupt:
        print("\nScan interrupted by user.")
    finally:
        print("Cleaning up GPIO...")
        GPIO.cleanup()
