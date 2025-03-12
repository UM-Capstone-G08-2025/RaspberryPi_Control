import serial
import RPi.GPIO as GPIO
import time
import threading
#from picamera2 import Picamera2
#from libcamera import controls
#import os
from legControl import LegControl

# Setup GPIO using BCM numbering for motor control
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Define GPIO pins for the motor driver (wheel system)
LEFT_MOTOR_BACKWARD = 19 #brown wire
LEFT_MOTOR_FORWARD = 26 #orange wire
RIGHT_MOTOR_FORWARD = 13 #green wire
RIGHT_MOTOR_BACKWARD = 6 #blue wire

# Define GPIO pins for the ultrasonic sensors
TRIG_LEFT = 23
ECHO_LEFT = 24
TRIG_RIGHT = 8
ECHO_RIGHT = 25

# Define GPIO pins for the LED toggle switch
BLUE_LED = 20  # Indicates wheel mode
GREEN_LED = 21  # Indicates leg mode
TOGGLE_SWITCH = 16  # Toggle switch pin

# Define a short delay to control measurement frequency
DELAY_TIME = 0.2  # seconds
TIMEOUT = 0.04  # 40ms timeout (~700cm max distance)

# Configure GPIO pins for motors (wheel system)
GPIO.setup(LEFT_MOTOR_FORWARD, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_BACKWARD, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_FORWARD, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_BACKWARD, GPIO.OUT)
GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)
GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)

# Configure GPIO pins for ultrasonic sensors
GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

# Configure GPIO pins for LED toggle switch
GPIO.setup(TOGGLE_SWITCH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(BLUE_LED, GPIO.OUT)
GPIO.setup(GREEN_LED, GPIO.OUT)

# Initialize the camera
# picam2 = Picamera2()
# os.system("v4l2-ctl --set-ctrl wide_dynamic_range=1 -d /dev/v4l-subdev0")
# print("Setting HDR to ON")
# picam2.start(show_preview=False)  # Disable preview to save resources
# picam2.set_controls({"AfMode": controls.AfModeEnum.Continuous, "AfSpeed": controls.AfSpeedEnum.Fast})

# Setup serial communication for UART
try:
    data = serial.Serial("/dev/ttyS0", 115200, timeout=1)  # Added timeout for better control
    print("Serial connection established.")
except serial.SerialException as e:
    print(f"Error initializing serial connection: {e}")
    exit(1)

# Global variables
intruder_detected = False  # intruder detection variable
mode_lock = threading.Lock()  # Lock for thread-safe mode switching
wheel_mode = True  # Default mode is wheel mode

# Initialize leg control
leg_control = LegControl()

def read_data():
    """
    Read class and bounding box coordinates from UART.
    """
    try:
        data_line = data.readline().decode('utf-8').strip()
        if "detected:" in data_line and "at" in data_line:
            class_part = data_line.split("detected:")[1].split("at")[0].strip()
            start = data_line.find('(') + 1
            end = data_line.find(')')
            coordinates = data_line[start:end].split(',')
            coordinates = list(map(int, coordinates))  # Convert coordinates to integers
            if len(coordinates) == 4:
                return (class_part, *coordinates)
    except (UnicodeDecodeError, ValueError) as e:
        print(f"Error reading data: {e}")
    return None

def get_distance(trig, echo):
    """
    Measure distance with timeout handling for a single ultrasonic sensor.
    """
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_time = time.time()
    stop_time = time.time()

    # Wait for echo to go low (start)
    timeout = start_time + TIMEOUT
    while GPIO.input(echo) == 0 and start_time < timeout:
        start_time = time.time()

    # Wait for echo to go high (end)
    timeout = start_time + TIMEOUT
    while GPIO.input(echo) == 1 and stop_time < timeout:
        stop_time = time.time()

    if stop_time - start_time >= TIMEOUT:
        return (None, None)  # Timeout occurred

    travel_time = stop_time - start_time
    distance_cm = (travel_time * 34300) / 2  # 343 m/s = 34300 cm/s
    distance_inch = distance_cm * 0.3937
    return round(distance_cm, 1), round(distance_inch, 1)

# Non-blocking wheel movement functions
def move_forward(duration):
    """
    Move the robot forward for a specified duration.
    """
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)
    time.sleep(duration)
    stop()

def move_backward(duration):
    """
    Move the robot backward for a specified duration.
    """
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)
    time.sleep(duration)
    stop()

def turn_right(duration):
    """
    Turn the robot left for a specified duration.
    """
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)
    time.sleep(duration)
    stop()

def turn_left(duration):
    """
    Turn the robot right for a specified duration.
    """
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)
    time.sleep(duration)
    stop()

def stop():
    """
    Stop all motors.
    """
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)

def cleanup():
    """
    Clean up GPIO pins.
    """
    GPIO.cleanup()
    leg_control.disconnect()

def take_picture():
    """
    Take a picture when an intruder is detected.
    """
    picam2.start_and_capture_files("intruder{:d}.jpg", num_files=1)
    print("Picture taken.")

# Non-blocking leg movement functions
def leg_walk_forward(duration):
    """
    Move the robot forward using legs.
    """
    leg_control.animationWalk()
    time.sleep(duration)
    leg_control.animationStand()

def leg_walk_backward(duration):
    """
    Move the robot backward using legs.
    """
    leg_control.animationWalk()
    time.sleep(duration)
    leg_control.animationStand()

def leg_turn_left(duration):
    """
    Turn the robot left using legs.
    """
    leg_control.animationTurnLeft()
    time.sleep(duration)
    leg_control.animationStand()

def leg_turn_right(duration):
    """
    Turn the robot right using legs.
    """
    leg_control.animationTurnRight()
    time.sleep(duration)
    leg_control.animationStand()

def leg_stop():
    """
    Stop leg movement.
    """
    leg_control.animationStand()

def follow_bounding_box(x, y, width, height):
    """
    Adjust movement based on bounding box coordinates.
    """
    center_x = x + width // 2
    frame_center_x = 320  # Assuming a frame width of 640 pixels
    threshold = 50  # Threshold for center alignment
    area = width * height  # Calculate the area of the bounding box

    if abs(center_x - frame_center_x) > threshold:
        if center_x < frame_center_x:
            print("Object to the left, turning left.")
            if wheel_mode:
                turn_left(0.3)
            else:
                leg_turn_left(0.3)
        else:
            print("Object to the right, turning right.")
            if wheel_mode:
                turn_right(0.3)
            else:
                leg_turn_right(0.3)
    else:
        if area > 50000:  # Example threshold for area
            print("Object is close, moving backward.")
            if wheel_mode:
                move_backward(0.3)
            else:
                leg_walk_backward(0.3)
        else:
            print("Object centered, moving forward.")
            if wheel_mode:
                move_forward(0.3)
            else:
                leg_walk_forward(0.3)

def read_uart_data():
    """
    Continuously reads the UART data and controls movement based on detected object.
    """
    global intruder_detected
    last_check_time = time.time()
    check_interval = 0.1  # Check every 0.1 seconds

    while True:
        current_time = time.time()
        if current_time - last_check_time >= check_interval:
            uart_data = read_data()
            if uart_data:
                obj_class, x, y, width, height = uart_data
                print(f"Class: {obj_class}, Bounding box: ({x}, {y}, {width}, {height})")

                # Control movement based on detected object
                if obj_class == "intruder":
                    follow_bounding_box(x, y, width, height)
                    intruder_detected = True  # Set the flag
                    #take_picture()  # Take a picture when an intruder is detected

            last_check_time = current_time

def read_ultrasonic_data():
    """
    Continuously reads the ultrasonic sensor data and controls movement based on proximity.
    """
    global intruder_detected
    last_check_time = time.time()
    check_interval = DELAY_TIME  # Check at the defined interval

    while True:
        current_time = time.time()
        if current_time - last_check_time >= check_interval:
            if not intruder_detected:  # Skip obstacle avoidance if an intruder is detected
                # Measure distances from both sensors
                left_cm, left_inch = get_distance(TRIG_LEFT, ECHO_LEFT)
                right_cm, right_inch = get_distance(TRIG_RIGHT, ECHO_RIGHT)

                # Handle potential timeouts
                if left_cm is not None and right_cm is not None:
                    print(f"Left: {left_cm} cm | Right: {right_cm} cm")

                    # Avoid obstacles based on sensor data
                    if left_cm <= 20 or right_cm <= 20:
                        print("Object too close, turning to avoid obstacle.")
                        if wheel_mode:
                            turn_right(0.6)
                        else:
                            leg_turn_right(0.6)

            last_check_time = current_time

def toggle_switch_control():
    """
    Control the robot based on the toggle switch state.
    """
    global wheel_mode
    previous_mode = wheel_mode  # Track the previous mode to detect changes
    while True:
        if GPIO.input(TOGGLE_SWITCH) == 0:
            GPIO.output(BLUE_LED, GPIO.HIGH)
            GPIO.output(GREEN_LED, GPIO.LOW)
            with mode_lock:
                wheel_mode = True
                if previous_mode != wheel_mode:
                    leg_control.disconnect()
                    print("Wheel mode activated.")
        else:
            GPIO.output(BLUE_LED, GPIO.LOW)
            GPIO.output(GREEN_LED, GPIO.HIGH)
            with mode_lock:
                wheel_mode = False
                if previous_mode != wheel_mode:
                    leg_control.connect()
                    print("Leg mode activated.")
        previous_mode = wheel_mode  # Update the previous mode
        time.sleep(0.1)

def main():
    # Create threads for UART, ultrasonic data reading, and toggle switch control
    uart_thread = threading.Thread(target=read_uart_data, daemon=True)
    ultrasonic_thread = threading.Thread(target=read_ultrasonic_data, daemon=True)
    toggle_thread = threading.Thread(target=toggle_switch_control, daemon=True)

    # Start the threads
    uart_thread.start()
    ultrasonic_thread.start()
    toggle_thread.start()

    try:
        # Keep the main thread running while the others handle their tasks
        uart_thread.join()
        ultrasonic_thread.join()
        toggle_thread.join()
    except KeyboardInterrupt:
        print("Exiting program...")
    finally:
        cleanup()

if __name__ == "__main__":
    main()