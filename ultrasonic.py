import RPi.GPIO as GPIO
import time

# Setup GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for Trigger and Echo
TRIG_PIN = 23
ECHO_PIN = 24

# Define a short delay to control measurement frequency
DELAY_TIME = 0.2  # seconds

# Configure GPIO pins
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance():
    """
    Measures the distance to a target using the HC-SR04 ultrasonic sensor.

    Returns:
        tuple: Distance in centimeters and inches.
    """
    # Send a 10 µs pulse to the trigger pin
    GPIO.output(TRIG_PIN, False)
    time.sleep(0.000002)  # 2 µs delay
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)  # 10 µs delay
    GPIO.output(TRIG_PIN, False)

    # Wait for echo start
    while GPIO.input(ECHO_PIN) == 0:
        pass
    start_time = time.time()

    # Wait for echo end
    while GPIO.input(ECHO_PIN) == 1:
        pass
    stop_time = time.time()

    # Calculate travel time and distance
    travel_time = stop_time - start_time
    distance_cm = (travel_time * 34400) / 2  # Speed of sound: 344 m/s
    distance_inch = distance_cm * 0.3937008  # Convert cm to inches

    return round(distance_cm, 1), round(distance_inch, 1)

try:
    print("HC-SR04 Ultrasonic Sensor Distance Measurement")
    print("Press Ctrl+C to exit.")

    while True:
        # Measure distance
        distance_cm, distance_inch = get_distance()
        print(f"Distance = {distance_inch} inches | {distance_cm} cm")

        # Short delay between measurements
        time.sleep(DELAY_TIME)

except KeyboardInterrupt:
    print("\nExiting program...")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up. Program terminated.")