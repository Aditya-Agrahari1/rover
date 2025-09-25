import RPi.GPIO as GPIO
from time import sleep

# Motor driver pins
IN1, IN2, ENA = 24, 23, 25
IN3, IN4, ENB = 17, 27, 22

GPIO.setmode(GPIO.BCM)

# Setup pins
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)

# Setup PWM on ENA and ENB for speed control (100 Hz)
pwmA = GPIO.PWM(ENA, 100)
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

def forward(t=2, speed=70):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    sleep(t)
    stop()

def backward(t=2, speed=70):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    sleep(t)
    stop()

def left(t=1, speed=70):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    sleep(t)
    stop()

def right(t=1, speed=70):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    sleep(t)
    stop()

try:
    # Example sequence
    forward(2)   # Move forward for 2 sec
    sleep(1)
    backward(2)  # Move backward for 2 sec
    sleep(1)
    left(1.5)    # Turn left
    sleep(1)
    right(1.5)   # Turn right
    sleep(1)

except KeyboardInterrupt:
    pass

finally:
    stop()
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()
