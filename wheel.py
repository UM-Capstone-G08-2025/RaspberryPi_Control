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

if __name__ == "__main__":
    setup()
    try:
        move_forward(2)
        time.sleep(1)
        move_backward(2)
        time.sleep(1)
        turn_left(1)
        time.sleep(1)
        turn_right(1)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
