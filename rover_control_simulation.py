# AgriSense Rover Control Script - HARDWARE MODE
# Version: 5.0 (Modern GPIOZero Library for Pi 5 Compatibility)
# Description: This script uses the modern gpiozero library to control the rover,
# ensuring compatibility with the Raspberry Pi 5. It performs the dual-scan
# mission with vision-based navigation and plant confirmation.

from gpiozero import AngularServo, Motor
import time
import requests
import cv2
import numpy as np

# -----------------------------------------------------------------------------
# --- CONFIGURATION & CALIBRATION (CRITICAL) ---
# -----------------------------------------------------------------------------

# -- Backend Server URL --
BACKEND_URL = "https://agrisense-backend-trdc.onrender.com/analyze"

# -- IP Camera URL --
IP_CAMERA_URL = "http://192.168.117.249:8080/video"

# -- GPIO Pin Configuration (BCM numbering) --
# NOTE: For gpiozero Motor, the pins are (forward, backward)
MOTOR_L_PINS = (24, 23) # (IN1, IN2)
MOTOR_R_PINS = (17, 27) # (IN3, IN4)
# NOTE: The ENA/ENB pins are now called 'enable' pins
MOTOR_L_ENABLE_PIN = 25
MOTOR_R_ENABLE_PIN = 22
# Servo
SERVO_PIN = 18

# -- Farm Dimensions --
CROPS_PER_ROW = 5
TOTAL_COLUMNS = 4

# -- Rover Calibration --
NINETY_DEGREE_TURN_TIME_S = 1.5
COLUMN_SHIFT_DISTANCE_M = 0.5
MOTOR_SPEED_MPS = 0.2
MOTOR_SPEED_PERCENT = 0.75 # Speed as a value from 0.0 to 1.0 (e.g., 0.75 = 75%)

# -- Vision Navigation Tuning --
GREEN_LOWER = np.array([35, 50, 50])
GREEN_UPPER = np.array([85, 255, 255])
FORWARD_TARGET_AREA = 30000
SIDE_CONFIRMATION_AREA = 5000

# -----------------------------------------------------------------------------
# --- SCRIPT INITIALIZATION ---
# -----------------------------------------------------------------------------
# Initialize devices using gpiozero
left_motor = Motor(forward=MOTOR_L_PINS[0], backward=MOTOR_L_PINS[1], enable=MOTOR_L_ENABLE_PIN)
right_motor = Motor(forward=MOTOR_R_PINS[0], backward=MOTOR_R_PINS[1], enable=MOTOR_R_ENABLE_PIN)
servo = AngularServo(SERVO_PIN, min_angle=-90, max_angle=90)

# --- HARDWARE FUNCTIONS (using gpiozero) ---

def setup_devices():
    """Initializes devices."""
    servo.angle = 0 # Center servo on start
    time.sleep(1)
    print("Servo and Motors initialized via gpiozero.")

def stop_motors():
    left_motor.stop()
    right_motor.stop()

def move_forward_continuous():
    left_motor.forward(speed=MOTOR_SPEED_PERCENT)
    right_motor.forward(speed=MOTOR_SPEED_PERCENT)

def move_forward(duration_s):
    print(f"Moving forward for {duration_s:.2f} seconds...")
    move_forward_continuous()
    time.sleep(duration_s)
    stop_motors()

def turn_right(duration_s):
    print(f"Turning right for {duration_s:.2f} seconds...")
    left_motor.forward(speed=MOTOR_SPEED_PERCENT)
    right_motor.backward(speed=MOTOR_SPEED_PERCENT)
    time.sleep(duration_s)
    stop_motors()

def turn_left(duration_s):
    print(f"Turning left for {duration_s:.2f} seconds...")
    left_motor.backward(speed=MOTOR_SPEED_PERCENT)
    right_motor.forward(speed=MOTOR_SPEED_PERCENT)
    time.sleep(duration_s)
    stop_motors()

# -----------------------------------------------------------------------------
# --- CORE LOGIC FUNCTIONS (No changes needed here) ---
# -----------------------------------------------------------------------------

def find_and_move_to_next_plant(video_capture):
    print("--- Vision Navigation: Searching for next plant ---")
    servo.angle = 0
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
    angle = -90 if side == 'left' else 90
    print(f"--- Scanning {side} (Col: {col}) ---")
    servo.angle = angle
    time.sleep(1)
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
    print("--- Starting AgriSense Farm Scan (V5 - GPIOZero) ---")
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
            scan_side(cap, row_index, left_col_index, 'left')
            scan_side(cap, row_index, right_col_index, 'right')
            if row_index < CROPS_PER_ROW - 1:
                find_and_move_to_next_plant(cap)
                time.sleep(1)
        if path_index < num_paths - 1:
            shift_to_next_path()
            time.sleep(2)
    cap.release()
    print("\n===== Farm Scan Complete =====")

if __name__ == "__main__":
    try:
        setup_devices()
        start_farm_scan()
    except KeyboardInterrupt:
        print("\nScan interrupted by user.")
    finally:
        print("Cleaning up devices...")
        # gpiozero handles cleanup automatically when the script exits

