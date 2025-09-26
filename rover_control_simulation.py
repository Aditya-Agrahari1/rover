import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
from time import sleep
import requests
from picamera2 import Picamera2
import threading
import json

# --- Backend & MQTT Configuration ---
AGRISENSE_BACKEND_URL = "https://agrisense-backend-trdc.onrender.com"
MQTT_HOSTNAME = "00ad4f388bdb4d1787d970ef423e0443.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_USERNAME = "agrisense-rover"
MQTT_PASSWORD = "Agrisense@1"
MQTT_COMMAND_TOPIC = "agrisense/rover/command"

# --- Hardware Configuration ---
IN1, IN2, ENA = 24, 23, 25
IN3, IN4, ENB = 17, 27, 22
picam2 = Picamera2()
camera_config = picam2.create_still_configuration()
picam2.configure(camera_config)
GPIO.setmode(GPIO.BCM)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
pwmA = GPIO.PWM(ENA, 100)
pwmB = GPIO.PWM(ENB, 100)
pwmA.start(0)
pwmB.start(0)

# --- Global State Variables ---
survey_active = False
survey_thread = None
survey_id = None
# *CHANGE:* Add a variable to store the language for the current survey.
current_language_code = "en" 

# --- Motor Control Functions ---
def stop_motors():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)

def move_forward(t=2, speed=70):
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
    global survey_active, survey_id, current_language_code
    
    print(f"--- Starting Survey Loop (ID: {survey_id}) ---")
    picam2.start()
    sleep(2)

    image_counter = 0
    while survey_active:
        print(f"Survey {survey_id}: Moving forward...")
        move_forward(t=2, speed=70)
        if not survey_active: break

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

    try:
        print(f"Notifying backend of survey completion for {survey_id} in {current_language_code}...")
        # *CHANGE:* Send the language code along with the survey ID.
        requests.post(
            f"{AGRISENSE_BACKEND_URL}/rover/survey_complete", 
            json={"survey_id": survey_id, "language_code": current_language_code}
        )
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
    global survey_active, survey_thread, survey_id, current_language_code
    
    # *CHANGE:* Parse the incoming message as JSON.
    try:
        payload = json.loads(msg.payload.decode())
        command = payload.get("command")
        lang_code = payload.get("language_code", "en")
    except (json.JSONDecodeError, AttributeError):
        # Fallback for simple "START"/"STOP" string messages
        command = msg.payload.decode()
        lang_code = "en"

    print(f"Received command: {command} for language: {lang_code}")

    if command == "start" and not survey_active:
        survey_active = True
        survey_id = f"survey_{int(time.time())}"
        current_language_code = lang_code
        survey_thread = threading.Thread(target=survey_loop)
        survey_thread.start()
    elif command == "stop" and survey_active:
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
        survey_active = False
        if survey_thread is not None:
            survey_thread.join()
        stop_motors()
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()
        print("Motors stopped and GPIO cleaned up.")
