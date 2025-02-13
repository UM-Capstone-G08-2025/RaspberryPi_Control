import RPi.GPIO as GPIO
blue_led = 20
green_led = 21
toggle = 16
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(toggle, GPIO.IN, pull_up_down = GPIO.PUD_DOWN) #set pin16 as input and pull pin down to 0V
GPIO.setup(blue_led, GPIO.OUT)
GPIO.setup(green_led, GPIO.OUT)
while True:
       if GPIO.input(toggle) == 0:
               GPIO.output(blue_led, GPIO.HIGH)
               GPIO.output(green_led, GPIO.LOW)
       else:
               GPIO.output(blue_led, GPIO.LOW)
               GPIO.output(green_led, GPIO.HIGH)