# AgriSense Rover Control Script - HARDWARE MODE
# Version: 4.1 (Final - Dual-Scan Vision Navigation)
# Description: This script controls a rover that travels between two crop rows.
# It uses a servo-mounted camera to scan plants on both the left and right,
# confirming plant presence with computer vision before sending data to the backend.

import RPi.GPIO as GPIO
from gpiozero import AngularServo
import time
import requests
import cv2
import numpy as np

# -----------------------------------------------------------------------------
# --- CONFIGURATION & CALIBRATION (CRITICAL) ---
# --- YOU MUST UPDATE THESE VALUES BEFORE RUNNING ---
# -----------------------------------------------------------------------------

# -- Backend Server URL --
# Replace with the IP address of the computer running your backend
BACKEND_URL = "https://agrisense-backend-trdc.onrender.com/analyze"

# -- IP Camera URL --
# Replace with the URL from your phone's IP Webcam app
IP_CAMERA_URL = "http://192.168.117.249:8080/video"

# -- GPIO Pin Configuration (BCM numbering) --
# Motors
MOTOR_L_IN1 = 24
MOTOR_L_IN2 = 23
MOTOR_L_ENA = 25
MOTOR_R_IN3 = 17
MOTOR_R_IN4 = 27
MOTOR_R_ENB = 22
# Servo
SERVO_PIN = 18 # Connect servo signal wire to this pin

# -- Farm Dimensions --
CROPS_PER_ROW = 5
TOTAL_COLUMNS = 4 # 4 columns means 3 paths to travel

# -- Rover Calibration (CALIBRATE THESE BY TESTING) --
NINETY_DEGREE_TURN_TIME_S = 1.5 # Time needed for a perfect 90-degree turn
COLUMN_SHIFT_DISTANCE_M = 0.5  # The distance between your crop columns
MOTOR_SPEED_MPS = 0.2          # Rover speed in meters/second

# -- Vision Navigation Tuning (CALIBRATE WITH vision_tuner.py) --
GREEN_LOWER = np.array([35, 50, 50]) # Lower bound for green in HSV
GREEN_UPPER = np.array([85, 255, 255])# Upper bound for green in HSV
FORWARD_TARGET_AREA = 30000          # Pixel area to stop at when moving forward
SIDE_CONFIRMATION_AREA = 5000        # Min pixel area to confirm a plant is present

# -----------------------------------------------------------------------------
# --- SCRIPT INITIALIZATION ---
# -----------------------------------------------------------------------------
pwm_l = None
pwm_r = None
servo = None

# --- HARDWARE FUNCTIONS ---

def setup_gpio_and_servo():
    """Initializes GPIO pins for motors and the servo."""
    global pwm_l, pwm_r, servo
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    motor_pins = [MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_ENA, MOTOR_R_IN3, MOTOR_R_IN4, MOTOR_R_ENB]
    for pin in motor_pins:
        GPIO.setup(pin, GPIO.OUT)
    pwm_l = GPIO.PWM(MOTOR_L_ENA, 100)
    pwm_r = GPIO.PWM(MOTOR_R_ENB, 100)
    pwm_l.start(75) # Set motor speed (0-100)
    pwm_r.start(75)
    print("GPIO setup complete.")
    servo = AngularServo(SERVO_PIN, min_angle=-90, max_angle=90)
    servo.angle = 0 # Center servo on start
    time.sleep(1)
    print("Servo setup complete.")

def stop_motors():
    GPIO.output(MOTOR_L_IN1, GPIO.LOW)
    GPIO.output(MOTOR_L_IN2, GPIO.LOW)
    GPIO.output(MOTOR_R_IN3, GPIO.LOW)
    GPIO.output(MOTOR_R_IN4, GPIO.LOW)

def move_forward_continuous():
    GPIO.output(MOTOR_L_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_L_IN2, GPIO.LOW)
    GPIO.output(MOTOR_R_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_R_IN4, GPIO.LOW)

def move_forward(duration_s):
    print(f"Moving forward for {duration_s:.2f} seconds...")
    move_forward_continuous()
    time.sleep(duration_s)
    stop_motors()

def turn_right(duration_s):
    print(f"Turning right for {duration_s:.2f} seconds...")
    # Left wheel forward, Right wheel backward for pivot turn
    GPIO.output(MOTOR_L_IN1, GPIO.HIGH)
    GPIO.output(MOTOR_L_IN2, GPIO.LOW)
    GPIO.output(MOTOR_R_IN3, GPIO.LOW)
    GPIO.output(MOTOR_R_IN4, GPIO.HIGH)
    time.sleep(duration_s)
    stop_motors()

def turn_left(duration_s):
    print(f"Turning left for {duration_s:.2f} seconds...")
    # Left wheel backward, Right wheel forward for pivot turn
    GPIO.output(MOTOR_L_IN1, GPIO.LOW)
    GPIO.output(MOTOR_L_IN2, GPIO.HIGH)
    GPIO.output(MOTOR_R_IN3, GPIO.HIGH)
    GPIO.output(MOTOR_R_IN4, GPIO.LOW)
    time.sleep(duration_s)
    stop_motors()

# -----------------------------------------------------------------------------
# --- CORE LOGIC FUNCTIONS ---
# -----------------------------------------------------------------------------

def find_and_move_to_next_plant(video_capture):
    """Uses computer vision to find the next plant IN FRONT and moves towards it."""
    print("--- Vision Navigation: Searching for next plant ---")
    servo.angle = 0 # Ensure camera is facing forward
    time.sleep(0.5)
    while True:
        ret, frame = video_capture.read()
        if not ret: continue
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            print(f"Plant detected ahead. Area: {area:.0f} (Target: {FORWARD_TARGET_AREA})")
            if area > FORWARD_TARGET_AREA:
                stop_motors()
                print("--- Plant reached. Stopping. ---")
                break
            else:
                move_forward_continuous()
        else:
            print("No plant detected ahead, moving forward...")
            move_forward_continuous()
        time.sleep(0.1)

def analyze_plant_at(frame, row, col):
    """Encodes and sends a single frame to the backend."""
    print(f"-> Analyzing confirmed plant at (Col: {col}, Row: {row}).")
    is_success, buffer = cv2.imencode(".jpg", frame)
    if not is_success:
        print("   [ERROR] Could not encode frame to JPEG.")
        return
    payload = {'language_code': 'en', 'row': row, 'col': col}
    files = {'image': ('image.jpg', buffer.tobytes(), 'image/jpeg')}
    try:
        print("   Sending image to backend server...")
        response = requests.post(BACKEND_URL, data=payload, files=files, timeout=90)
        response.raise_for_status()
        print(f"   Success! Backend responded with status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"   [ERROR] Failed to send image to backend: {e}")

def scan_side(video_capture, row, col, side):
    """Rotates camera, confirms plant presence, and calls analysis function."""
    angle = -90 if side == 'left' else 90
    print(f"--- Scanning {side} (Col: {col}) ---")
    servo.angle = angle
    time.sleep(1) # Wait for servo to settle
    
    ret, frame = video_capture.read()
    if not ret:
        print(f"   [ERROR] Could not capture frame for {side} scan.")
        return
        
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > SIDE_CONFIRMATION_AREA:
            print(f"   Plant confirmed on {side}. Area: {area:.0f}")
            analyze_plant_at(frame, row, col)
        else:
            print(f"   No plant confirmed on {side}. Contour too small. Area: {area:.0f}")
    else:
        print(f"   No plant confirmed on {side}. No contours found.")

def shift_to_next_path():
    """Moves the rover sideways to the next path between columns."""
    print("--- Shifting to the next path ---")
    shift_duration = COLUMN_SHIFT_DISTANCE_M / MOTOR_SPEED_MPS
    turn_right(NINETY_DEGREE_TURN_TIME_S)
    move_forward(shift_duration)
    turn_left(NINETY_DEGREE_TURN_TIME_S)
    print("--- Aligned with the next path ---")

# -----------------------------------------------------------------------------
# --- MAIN EXECUTION ---
# -----------------------------------------------------------------------------

def start_farm_scan():
    print("--- Starting AgriSense Farm Scan (DUAL SCAN MODE V4.1) ---")
    cap = cv2.VideoCapture(IP_CAMERA_URL)
    if not cap.isOpened():
        print(f"[FATAL ERROR] Could not open video stream at {IP_CAMERA_URL}")
        return
    print(f"Successfully connected to IP Camera at {IP_CAMERA_URL}")
    time.sleep(2)
    
    num_paths = TOTAL_COLUMNS - 1
    
    for path_index in range(num_paths):
        left_col_index = path_index
        right_col_index = path_index + 1
        print(f"\n===== SCANNING PATH {path_index} (Between Col {left_col_index} and {right_col_index}) =====")
        
        for row_index in range(CROPS_PER_ROW):
            # At each stop, scan both left and right sides
            scan_side(cap, row_index, left_col_index, 'left')
            scan_side(cap, row_index, right_col_index, 'right')
            
            # After scanning the pair, move to the next plant pair
            if row_index < CROPS_PER_ROW - 1:
                find_and_move_to_next_plant(cap)
                time.sleep(1) # Pause after reaching plant
        
        # After finishing a path, shift to the next one
        if path_index < num_paths - 1:
            shift_to_next_path()
            time.sleep(2)
            
    cap.release()
    servo.close()
    print("\n===== Farm Scan Complete =====")

if __name__ == "__main__":
    try:
        setup_gpio_and_servo()
        start_farm_scan()
    except KeyboardInterrupt:
        print("\nScan interrupted by user.")
    finally:
        print("Cleaning up GPIO...")
        # A good practice to ensure servo is detached and motors are stopped
        if servo:
            servo.close()
        GPIO.cleanup()

