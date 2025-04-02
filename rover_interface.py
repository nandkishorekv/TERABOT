#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import threading
from flask import Flask, render_template, request, jsonify

# ================================
# TERRABOT: AI-Driven Rover
# ================================

# GPIO Pins for L298N Motor Driver
IN1, IN2 = 16, 20     # Left Motor Control
IN3, IN4 = 21, 26     # Right Motor Control
ENA, ENB = 13, 6      # PWM Speed Control

# GPIO Pins for HC-SR04 Ultrasonic Sensor
TRIG, ECHO = 4, 17

# GPIO Pins for Servo Motors
SERVO_PIN = 18        # Obstacle Avoidance Servo
PAN_PIN = 25          # Pan Servo
TILT_PIN = 8          # Tilt Servo

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup([SERVO_PIN, PAN_PIN, TILT_PIN], GPIO.OUT)

# PWM Setup
pwm_ENA, pwm_ENB = GPIO.PWM(ENA, 100), GPIO.PWM(ENB, 100)  # 100Hz PWM Frequency
pwm_ENA.start(0)
pwm_ENB.start(0)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz for Servo
servo_pwm.start(0)
pan_pwm = GPIO.PWM(PAN_PIN, 50)
pan_pwm.start(0)
tilt_pwm = GPIO.PWM(TILT_PIN, 50)
tilt_pwm.start(0)

# ================================
# Motor and Sensor Functions
# ================================
def set_motor(left_speed, right_speed):
    left_speed = max(-100, min(100, left_speed))
    right_speed = max(-100, min(100, right_speed))
    GPIO.output(IN1, left_speed > 0)
    GPIO.output(IN2, left_speed < 0)
    GPIO.output(IN3, right_speed > 0)
    GPIO.output(IN4, right_speed < 0)
    pwm_ENA.ChangeDutyCycle(abs(left_speed))
    pwm_ENB.ChangeDutyCycle(abs(right_speed))

def get_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start_time = time.time()
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
    stop_time = time.time()
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
    return (stop_time - start_time) * 17150

def set_servo_angle(pwm, angle):
    duty = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)

# ================================
# Obstacle Handling
# ================================
def detect_obstacle():
    """Returns True if an obstacle is detected within 40 cm."""
    return get_distance() < 40

def scan_best_path():
    """Scans left and right and returns the best direction."""
    set_servo_angle(servo_pwm, 0)
    left_distance = get_distance()
    set_servo_angle(servo_pwm, 180)
    right_distance = get_distance()
    set_servo_angle(servo_pwm, 90)  # Reset to center
    if left_distance < 40 and right_distance < 40:
        return "backward"
    return "left" if left_distance > right_distance else "right"

def obstacle_avoidance():
    """Autonomous mode obstacle avoidance logic."""
    if detect_obstacle():
        set_motor(0, 0)  # Stop motors
        time.sleep(0.5)
        direction = scan_best_path()
        if direction == "backward":
            set_motor(-40, -40)  # Move backward
            time.sleep(1)
        elif direction == "left":
            set_motor(-30, 60)  # Turn left
            time.sleep(1)
        elif direction == "right":
            set_motor(60, -30)  # Turn right
            time.sleep(1)
        return
    set_motor(50, 50)  # Move forward

def manual_obstacle_avoidance():
    """Handles obstacle detection in manual mode."""
    if detect_obstacle():
        set_motor(0, 0)  # Stop motors
        time.sleep(0.5)
        direction = scan_best_path()
        if direction == "backward":
            set_motor(-40, -40)  # Move backward
            time.sleep(1)
        elif direction == "left":
            set_motor(-30, 60)  # Turn left
            time.sleep(1)
        elif direction == "right":
            set_motor(60, -30)  # Turn right
            time.sleep(1)
        set_motor(0, 0)  # Wait for user input
        return True
    return False

# ================================
# Flask Web Interface
# ================================
app = Flask(__name__)
mode = "manual"
autonomous_thread = None
stop_autonomous = threading.Event()

@app.route("/")
def index():
    return render_template("index.html", mode=mode)

@app.route("/set_mode", methods=["POST"])
def set_mode():
    global mode, autonomous_thread
    new_mode = request.json.get("mode")
    if new_mode not in ["manual", "autonomous"]:
        return jsonify({"error": "Invalid mode"}), 400
    if new_mode == "autonomous":
        mode = "autonomous"
        stop_autonomous.clear()
        if autonomous_thread is None or not autonomous_thread.is_alive():
            autonomous_thread = threading.Thread(target=obstacle_avoidance, daemon=True)
            autonomous_thread.start()
    else:
        mode = "manual"
        stop_autonomous.set()
        set_motor(0, 0)
    return jsonify({"mode": mode})

@app.route("/manual/move", methods=["POST"])
def manual_move():
    if mode != "manual":
        return jsonify({"error": "Not in manual mode"}), 400
    direction = request.json.get("direction")
    if manual_obstacle_avoidance():
        return jsonify({"error": "Obstacle detected! Adjusted direction. Awaiting user input."}), 400
    moves = {
        "forward": (50, 50),
        "backward": (-50, -50),
        "left": (-30, 60),
        "right": (60, -30),
        "stop": (0, 0)
    }
    if direction not in moves:
        return jsonify({"error": "Invalid direction"}), 400
    set_motor(*moves[direction])
    return jsonify({"status": "ok"})

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000, debug=False)
    except KeyboardInterrupt:
        pwm_ENA.stop()
        pwm_ENB.stop()
        servo_pwm.stop()
        pan_pwm.stop()
        tilt_pwm.stop()
        GPIO.cleanup()
