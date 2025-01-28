import RPi.GPIO as GPIO
import time

# Define GPIO pins for the motor driver
LEFT_MOTOR_FORWARD = 19 #brown wire
LEFT_MOTOR_BACKWARD = 26 #orange wire
RIGHT_MOTOR_FORWARD = 13 #green wire
RIGHT_MOTOR_BACKWARD = 6 #blue wire

def setup():
    """
    Configure GPIO pins for motor control.
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(LEFT_MOTOR_FORWARD, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_BACKWARD, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_FORWARD, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_BACKWARD, GPIO.OUT)
    GPIO.output(LEFT_MOTOR_BACKWARD,GPIO.LOW)
    GPIO.output(LEFT_MOTOR_FORWARD,GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD,GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD,GPIO.LOW)

def move_forward(duration):
    """
    Move the robot forward for a specified duration.
    """
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.LOW)
    time.sleep(duration)
    stop()

def move_backward(duration):
    """
    Move the robot backward for a specified duration.
    """
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    time.sleep(duration)
    stop()

def turn_left(duration):
    """
    Turn the robot left for a specified duration.
    """
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.LOW)
    GPIO.output(RIGHT_MOTOR_BACKWARD, GPIO.HIGH)
    time.sleep(duration)
    stop()

def turn_right(duration):
    """
    Turn the robot right for a specified duration.
    """
    GPIO.output(LEFT_MOTOR_FORWARD, GPIO.HIGH)
    GPIO.output(RIGHT_MOTOR_FORWARD, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_BACKWARD, GPIO.HIGH)
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

def control_robot():
    """
    Take user input from the terminal to control the robot.
    """
    while True:
        # Prompt user for input
        command = input("Enter a command (f: forward, b: backward, l: left, r: right, s: stop, q: quit): ").strip().lower()

        if command == "f":
            move_forward(3)  # Move forward for 2 seconds
        elif command == "b":
            move_backward(2)  # Move backward for 2 seconds
        elif command == "l":
            turn_left(1)  # Turn left for 1 second
        elif command == "r":
            turn_right(1)  # Turn right for 1 second
        elif command == "s":
            stop()  # Stop the robot
        elif command == "q":
            break  # Exit the loop and terminate the program
        else:
            print("Invalid command! Please try again.")

if __name__ == "__main__":
    setup()
    try:
        control_robot()  # Start accepting user input for control
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
