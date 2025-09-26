import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from time import sleep
import requests
from picamera2 import Picamera2
import threading

# --- Backend & MQTT Configuration ---
AGRISENSE_BACKEND_URL = "https://agrisense-backend-trdc.onrender.com"
MQTT_HOSTNAME = "00ad4f388bdb4d1787d970ef423e0443.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_USERNAME = "agrisense-rover"
MQTT_PASSWORD = "Agrisense@1"
MQTT_COMMAND_TOPIC = "agrisense/rover/command"

# --- Hardware Configuration (from your provided logic) ---
# Motor driver pins
IN1, IN2, ENA = 24, 23, 25
IN3, IN4, ENB = 17, 27, 22

# Camera setup
picam2 = Picamera2()
camera_config = picam2.create_still_configuration()
picam2.configure(camera_config)

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
pwmA = GPIO.PWM(ENA, 100)
pwmB = GPIO.PWM(ENB, 100)
pwmA.start(0)
pwmB.start(0)

# --- Global State Variables ---
# These flags control the survey loop.
survey_active = False
survey_thread = None
survey_id = None

# --- Motor Control Functions ---
def stop_motors():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)

def move_forward(t=2, speed=70):
    # This function now matches your specific pin logic for forward movement
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    sleep(t)
    stop_motors()

# --- Image & Network Functions ---
def send_image_to_backend(file_path, current_survey_id):
    """Posts an image file to the backend."""
    url = f"{AGRISENSE_BACKEND_URL}/rover/upload_image"
    try:
        with open(file_path, 'rb') as f:
            files = {'image': (file_path, f, 'image/jpeg')}
            data = {'survey_id': current_survey_id}
            response = requests.post(url, files=files, data=data)
            response.raise_for_status()
        print(f"Successfully sent {file_path} for survey {current_survey_id}")
        return True
    except requests.exceptions.RequestException as e:
        print(f"Failed to send image: {e}")
        return False

# --- Main Survey Loop ---
def survey_loop():
    """The main loop for the rover's operation."""
    global survey_active, survey_id
    
    print(f"--- Starting Survey Loop (ID: {survey_id}) ---")
    picam2.start()
    sleep(2) # Camera warm-up

    image_counter = 0
    # The loop continues as long as the survey_active flag is True.
    while survey_active:
        print(f"Survey {survey_id}: Moving forward...")
        move_forward(t=2, speed=70)

        # Check again in case a 'STOP' command was received during movement
        if not survey_active:
            break

        image_counter += 1
        file_path = f"/home/pi/Pictures/{survey_id}img{image_counter}.jpg"
        print(f"Survey {survey_id}: Capturing image {image_counter}...")
        picam2.capture_file(file_path)
        
        send_image_to_backend(file_path, survey_id)

        print(f"Survey {survey_id}: Waiting for 2 seconds...")
        sleep(2)

    picam2.stop()
    stop_motors()
    print(f"--- Survey Loop Stopped (ID: {survey_id}) ---")

    # Final step: Notify the backend that this survey is complete.
    try:
        print(f"Notifying backend of survey completion for {survey_id}...")
        requests.post(f"{AGRISENSE_BACKEND_URL}/rover/survey_complete", json={"survey_id": survey_id})
    except Exception as e:
        print(f"Failed to notify backend: {e}")


# --- MQTT Handler ---
def on_connect(client, userdata, flags, rc, properties):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(MQTT_COMMAND_TOPIC)
    else:
        print(f"Failed to connect, return code {rc}\n")

def on_message(client, userdata, msg):
    """This function is the main controller, reacting to app commands."""
    global survey_active, survey_thread, survey_id
    command = msg.payload.decode()
    print(f"Received command: {command}")

    if command == "START" and not survey_active:
        survey_active = True
        survey_id = f"survey_{int(time.time())}"
        # Start the survey_loop in a new thread to keep the MQTT client responsive.
        survey_thread = threading.Thread(target=survey_loop)
        survey_thread.start()
    elif command == "STOP" and survey_active:
        # Simply setting this flag to False will cause the loop in the thread to exit.
        survey_active = False
        print("STOP signal received. The survey will end after its current action.")

# --- Main Execution ---
if _name_ == '_main_':
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.tls_set(tls_version=mqtt.ssl.PROTOCOL_TLS)
    client.on_connect = on_connect
    client.on_message = on_message
    
    try:
        print(f"Connecting to {MQTT_HOSTNAME}...")
        client.connect(MQTT_HOSTNAME, MQTT_PORT, 60)
        client.loop_forever()
    except KeyboardInterrupt:
        print("Script interrupted by user.")
    finally:
        # This ensures that motors stop and GPIO pins are cleaned up safely.
        survey_active = False # Stop the survey loop if it's running
        if survey_thread is not None:
            survey_thread.join() # Wait for the thread to finish
        stop_motors()
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()
        print("Motors stopped and GPIO cleaned up.")
