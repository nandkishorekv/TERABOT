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
TRIG, ECHO = 22, 27   # Updated TRIG and ECHO pins

# GPIO Pins for Servo Motors
SERVO_PIN = 18        # Obstacle Avoidance Servo
PAN_PIN, TILT_PIN = 25, 8  # Pan & Tilt Servos

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup([SERVO_PIN, PAN_PIN, TILT_PIN], GPIO.OUT)

# Initialize PWM
pwm_ENA = GPIO.PWM(ENA, 100)
pwm_ENB = GPIO.PWM(ENB, 100)
pwm_ENA.start(0)
pwm_ENB.start(0)

servo_pwm = GPIO.PWM(SERVO_PIN, 50)
servo_pwm.start(0)
pan_pwm = GPIO.PWM(PAN_PIN, 50)
pan_pwm.start(0)
tilt_pwm = GPIO.PWM(TILT_PIN, 50)
tilt_pwm.start(0)

# ================================
# Motor and Sensor Functions
# ================================

def set_motor(left_speed, right_speed):
    """ Control motors with given speeds (-100 to 100). """
    left_speed = max(-100, min(100, left_speed))
    right_speed = max(-100, min(100, right_speed))
    
    GPIO.output(IN1, left_speed > 0)
    GPIO.output(IN2, left_speed <= 0)
    GPIO.output(IN3, right_speed > 0)
    GPIO.output(IN4, right_speed <= 0)

    pwm_ENA.ChangeDutyCycle(abs(left_speed))
    pwm_ENB.ChangeDutyCycle(abs(right_speed))

def get_distance():
    """ Get distance measurement from ultrasonic sensor. """
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    start_time, stop_time = time.time(), time.time()
    
    timeout = time.time() + 0.02  # 20ms timeout
    while GPIO.input(ECHO) == 0:
        start_time = time.time()
        if time.time() > timeout:
            return -1  # Return -1 on timeout

    timeout = time.time() + 0.02
    while GPIO.input(ECHO) == 1:
        stop_time = time.time()
        if time.time() > timeout:
            return -1  # Return -1 on timeout

    distance = (stop_time - start_time) * 17150
    return round(distance, 2)

def set_servo_angle(pwm, angle):
    """ Set the servo motor to a specified angle. """
    duty = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.3)  # Reduce delay to improve responsiveness
    pwm.ChangeDutyCycle(0)

# ================================
# Obstacle Avoidance & Autonomous Mode
# ================================

def obstacle_avoidance():
    """ Check obstacles and decide movement direction. """
    distance = get_distance()
    if distance == -1 or distance >= 40:
        return None

    set_motor(0, 0)
    time.sleep(0.3)

    # Check left
    set_servo_angle(servo_pwm, 0)
    left_distance = get_distance()

    # Check right
    set_servo_angle(servo_pwm, 180)
    right_distance = get_distance()

    # Reset to center
    set_servo_angle(servo_pwm, 90)

    if left_distance < 40 and right_distance < 40:
        return "backward"
    return "left" if left_distance > right_distance else "right"

def autonomous_control():
    """ Autonomous obstacle avoidance control loop. """
    while not stop_autonomous.is_set():
        direction = obstacle_avoidance()
        if direction:
            if direction == "backward":
                set_motor(-40, -40)
            elif direction == "left":
                set_motor(-20, 50)
            elif direction == "right":
                set_motor(50, -20)
            time.sleep(0.8)
        else:
            set_motor(50, 50)
            time.sleep(0.1)
    set_motor(0, 0)

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
    """ Switch between manual and autonomous modes. """
    global mode, autonomous_thread
    new_mode = request.json.get("mode")
    
    if new_mode not in ["manual", "autonomous"]:
        return jsonify({"error": "Invalid mode"}), 400

    if new_mode == "autonomous":
        mode = "autonomous"
        stop_autonomous.clear()
        if autonomous_thread is None or not autonomous_thread.is_alive():
            autonomous_thread = threading.Thread(target=autonomous_control, daemon=True)
            autonomous_thread.start()
    else:
        mode = "manual"
        stop_autonomous.set()
        set_motor(0, 0)

    return jsonify({"mode": mode})

@app.route("/manual/move", methods=["POST"])
def manual_move():
    """ Handle manual movement commands. """
    if mode != "manual":
        return jsonify({"error": "Not in manual mode"}), 400

    direction = request.json.get("direction")

    if direction == "forward":
        obstacle = obstacle_avoidance()
        if obstacle:
            return jsonify({"status": "obstacle_detected", "action": obstacle})

        set_motor(50, 50)

    elif direction == "backward":
        set_motor(-50, -50)
    elif direction == "left":
        set_motor(-30, 60)
    elif direction == "right":
        set_motor(60, -30)
    elif direction == "stop":
        set_motor(0, 0)
    else:
        return jsonify({"error": "Invalid direction"}), 400

    return jsonify({"status": "ok"})

@app.route("/pan_tilt", methods=["POST"])
def pan_tilt():
    """ Handle pan-tilt servo control. """
    pan_angle = request.json.get("pan")
    tilt_angle = request.json.get("tilt")

    if pan_angle is not None:
        set_servo_angle(pan_pwm, pan_angle)
    if tilt_angle is not None:
        set_servo_angle(tilt_pwm, tilt_angle)

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
