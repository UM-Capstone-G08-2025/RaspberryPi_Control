import RPi.GPIO as GPIO
import time

# Setup GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for sensors
TRIG_LEFT = 23
ECHO_LEFT = 24
TRIG_RIGHT = 8
ECHO_RIGHT = 25

DELAY_TIME = 0.2  # seconds
TIMEOUT = 0.04  # 40ms timeout (~700cm max distance)

# Configure GPIO pins
GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)

def get_distance(trig, echo):
    """Measure distance with timeout handling."""
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

try:
    print("Monitoring both sensors...")
    while True:
        # Measure left sensor
        left_cm, left_inch = get_distance(TRIG_LEFT, ECHO_LEFT)
        # Measure right sensor
        right_cm, right_inch = get_distance(TRIG_RIGHT, ECHO_RIGHT)

        # Handle potential timeouts
        left_str = f"{left_cm} cm | {left_inch} in" if left_cm is not None else "Timeout"
        right_str = f"{right_cm} cm | {right_inch} in" if right_cm is not None else "Timeout"
        
        print(f"Left: {left_str}  ||  Right: {right_str}")
        time.sleep(DELAY_TIME)

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
