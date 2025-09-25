# AgriSense Rover Control Script - SIMULATION MODE
# Version: 1.0
# Description: This script simulates the AgriSense rover's functions.
# It connects to a mobile IP camera, captures images, and sends them to a backend
# for analysis, while simulating the rover's movements without requiring motor hardware.

# No longer needed for simulation, but good to keep the import commented
# import RPi.GPIO as GPIO
import time
import requests
import cv2
import numpy as np

# -----------------------------------------------------------------------------
# --- CONFIGURATION & CALIBRATION ---
# --- Adjust these values for your specific setup ---
# -----------------------------------------------------------------------------

# -- Backend Server URL --
# IMPORTANT: Replace with the actual IP address of the computer running your backend server
BACKEND_URL = "https://agrisense-backend-trdc.onrender.com/analyze"

# -- IP Camera URL --
# IMPORTANT: Replace with the URL from your IP Webcam app on your phone
# It often ends in /video or /shot.jpg
IP_CAMERA_URL = "http://192.168.117.249:8080/video"

# -- Farm Dimensions (Provided by Farmer) --
ROW_LENGTH_M = 10.0      # Length of one crop row in meters
CROPS_PER_ROW = 5        # Number of plants in a single row
TOTAL_COLUMNS = 3        # Total number of columns to scan

# -- Rover Movement Calibration (Used for timing the simulation) --
MOTOR_SPEED_MPS = 0.2    # Assumed speed of the rover in meters per second
NINETY_DEGREE_TURN_TIME_S = 1.5 # Time in seconds for a 90-degree turn
COLUMN_SHIFT_DISTANCE_M = 0.5 # The distance between two crop columns in meters

# -----------------------------------------------------------------------------
# --- SCRIPT INITIALIZATION ---
# -----------------------------------------------------------------------------

# Calculate distance between plants to determine movement time
if CROPS_PER_ROW > 1:
    DISTANCE_PER_CROP_M = ROW_LENGTH_M / (CROPS_PER_ROW - 1)
else:
    DISTANCE_PER_CROP_M = 0

# --- MOCKED HARDWARE FUNCTIONS (SIMULATION) ---
# These functions print actions and pause instead of controlling motors.

def setup_gpio():
    """Simulates GPIO pin setup."""
    print("[SIM] GPIO setup skipped (Simulation Mode).")

def move_forward(duration_s):
    """Simulates moving the rover forward."""
    print(f"[SIM] Moving forward for {duration_s:.2f} seconds...")
    time.sleep(duration_s)
    print("[SIM] Move complete.")

def turn_right(duration_s):
    """Simulates turning the rover right."""
    print(f"[SIM] Turning right for {duration_s:.2f} seconds...")
    time.sleep(duration_s)
    print("[SIM] Turn complete.")

def turn_left(duration_s):
    """Simulates turning the rover left."""
    print(f"[SIM] Turning left for {duration_s:.2f} seconds...")
    time.sleep(duration_s)
    print("[SIM] Turn complete.")

# -----------------------------------------------------------------------------
# --- CORE LOGIC FUNCTIONS ---
# -----------------------------------------------------------------------------

def move_forward_one_plant():
    """Calculates the required time and simulates moving to the next plant."""
    if MOTOR_SPEED_MPS == 0:
        print("Error: Motor speed is not calibrated (0). Cannot simulate movement.")
        return
    move_duration = DISTANCE_PER_CROP_M / MOTOR_SPEED_MPS
    move_forward(move_duration)

def shift_to_next_column():
    """Simulates the sequence of moves to align with the next column."""
    print("--- Shifting to the next column ---")
    shift_duration = COLUMN_SHIFT_DISTANCE_M / MOTOR_SPEED_MPS
    turn_right(NINETY_DEGREE_TURN_TIME_S)
    move_forward(shift_duration)
    turn_left(NINETY_DEGREE_TURN_TIME_S)
    print("--- Aligned with the next column ---")

def analyze_plant_at(video_capture, row, col):
    """Captures an image from the IP camera and sends it to the backend."""
    print(f"-> Analyzing plant at (Col: {col}, Row: {row}).")
    print("   Capturing frame from IP camera...")

    # Grab a single frame from the video stream
    ret, frame = video_capture.read()
    if not ret:
        print("   [ERROR] Could not read frame from video stream. Check camera connection.")
        return

    # Encode the captured frame to JPEG format in memory
    is_success, buffer = cv2.imencode(".jpg", frame)
    if not is_success:
        print("   [ERROR] Could not encode frame to JPEG.")
        return

    image_bytes = buffer.tobytes()

    # Prepare data for the POST request
    payload = {'language_code': 'en', 'row': row, 'col': col}
    files = {'image': ('image.jpg', image_bytes, 'image/jpeg')}

    try:
        print("   Sending image to backend server...")
        response = requests.post(BACKEND_URL, data=payload, files=files, timeout=30)
        response.raise_for_status() # Raise an exception for bad status codes (4xx or 5xx)
        print(f"   Success! Backend responded with status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"   [ERROR] Failed to send image to backend: {e}")

# -----------------------------------------------------------------------------
# --- MAIN EXECUTION ---
# -----------------------------------------------------------------------------

def start_farm_scan():
    """Orchestrates the entire simulated farm scan."""
    print("--- Starting AgriSense Farm Scan (SIMULATION MODE) ---")

    # Initialize video capture from the IP Camera
    cap = cv2.VideoCapture(IP_CAMERA_URL)
    if not cap.isOpened():
        print(f"[FATAL ERROR] Could not open video stream at {IP_CAMERA_URL}")
        print("Please check the URL and ensure your phone's IP camera app is running.")
        return

    print(f"Successfully connected to IP Camera at {IP_CAMERA_URL}")
    time.sleep(2) # Give camera stream time to stabilize

    for col in range(TOTAL_COLUMNS):
        print(f"\n===== SCANNING COLUMN {col} =====")
        for row in range(CROPS_PER_ROW):
            analyze_plant_at(cap, row, col)

            # Simulate moving to the next plant if it's not the last one in the row
            if row < CROPS_PER_ROW - 1:
                move_forward_one_plant()
                time.sleep(1) # Brief pause between plants

        # Simulate shifting to the next column if it's not the final column
        if col < TOTAL_COLUMNS - 1:
            shift_to_next_column()
            time.sleep(2) # Brief pause before starting next column

    cap.release() # Release the video capture object
    print("\n===== Farm Scan Complete =====")

if __name__ == "__main__":
    try:
        setup_gpio()
        start_farm_scan()
    except KeyboardInterrupt:
        print("\nScan interrupted by user.")
    finally:
        # GPIO.cleanup() is not needed in simulation mode
        print("Simulation finished. Exiting script.")

