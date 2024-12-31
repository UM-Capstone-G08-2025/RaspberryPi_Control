import RPi.GPIO as GPIO
import time

# Pin configuration
BUTTON_PIN = 23
LED_PIN = 18 

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Pull-down resistor
GPIO.setup(LED_PIN, GPIO.OUT)

# Initial state
current_mode = 0  # 0 = Don't blink too fast , 1 = Blinking faster

print("Press the button to toggle between behaviors.")

# Call back function on interrupt
def toggle_mode(channel):
    global current_mode
    current_mode = (current_mode + 1) % 2
    print(f"Toggled to mode {current_mode}")

# Interrupt on press of button
GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=toggle_mode, bouncetime=300)

try:
    while True:
        if current_mode == 0:
            # Don't bink too fast
            GPIO.output(LED_PIN, True)
            time.sleep(0.5)
            GPIO.output(LED_PIN, False)
            time.sleep(0.5)
            
            ### Maybe Wheel Control Code function calls Here
            
        elif current_mode == 1:
            # Blink faster
            GPIO.output(LED_PIN, True)
            time.sleep(0.2)
            GPIO.output(LED_PIN, False)
            time.sleep(0.2)
            
            ### Maybe Leg Control Code function calls Here
            
except:
    print('You terminated me')
    
finally:
    GPIO.cleanup() 
