# Python Script
# https://www.electronicshub.org/raspberry-pi-l298n-interface-tutorial-control-dc-motor-l298n-raspberry-

# TO DO - Modify code to work for all wheels
import RPi.GPIO as GPIO          
from time import sleep

in1_1 = 12
in2_1 = 16
in1_2 = 18
in2_2 = 23
in3_1 = 20
in4_1 = 21
in3_2 = 24
in4_2 = 25
en = 25
temp1=1

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1_1,GPIO.OUT)
GPIO.setup(in2_1,GPIO.OUT)
GPIO.setup(in1_2,GPIO.OUT)
GPIO.setup(in2_2,GPIO.OUT)
GPIO.setup(in3_1,GPIO.OUT)
GPIO.setup(in4_1,GPIO.OUT)
GPIO.setup(in3_2,GPIO.OUT)
GPIO.setup(in4_2,GPIO.OUT)
p=GPIO.PWM(en,1000)

p.start(25)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
print("\n")    

while(1):

    x=input()
    
    if x=='r':
        print("run")
        if(temp1==1):
         GPIO.output(in1_1,GPIO.HIGH)
         GPIO.output(in2_1,GPIO.LOW)
         GPIO.output(in1_2,GPIO.HIGH)
         GPIO.output(in2_2,GPIO.LOW)
         GPIO.output(in3_1,GPIO.HIGH)
         GPIO.output(in4_1,GPIO.LOW)
         GPIO.output(in3_2,GPIO.HIGH)
         GPIO.output(in4_2,GPIO.LOW)
         print("forward")
         x='z'
        else:
         GPIO.output(in1_1,GPIO.LOW)
         GPIO.output(in2_1,GPIO.HIGH)
         GPIO.output(in1_2,GPIO.LOW)
         GPIO.output(in2_2,GPIO.HIGH)
         GPIO.output(in3_1,GPIO.LOW)
         GPIO.output(in4_1,GPIO.HIGH)
         GPIO.output(in3_2,GPIO.LOW)
         GPIO.output(in4_2,GPIO.HIGH)
         print("backward")
         x='z'


    elif x=='s':
        print("stop")
        GPIO.output(in1_1,GPIO.LOW)
        GPIO.output(in2_1,GPIO.LOW)
        GPIO.output(in1_2,GPIO.LOW)
        GPIO.output(in2_2,GPIO.LOW)
        GPIO.output(in3_1,GPIO.LOW)
        GPIO.output(in4_1,GPIO.LOW)
        GPIO.output(in3_2,GPIO.LOW)
        GPIO.output(in4_2,GPIO.LOW)
        x='z'

    # elif x=='f':
        # print("forward")
        # GPIO.output(in1,GPIO.HIGH)
        # GPIO.output(in2,GPIO.LOW)
        # temp1=1
        # x='z'

    # elif x=='b':
        # print("backward")
        # GPIO.output(in1,GPIO.LOW)
        # GPIO.output(in2,GPIO.HIGH)
        # temp1=0
        # x='z'

    # elif x=='l':
        # print("low")
        # p.ChangeDutyCycle(25)
        # x='z'

    # elif x=='m':
        # print("medium")
        # p.ChangeDutyCycle(50)
        # x='z'

    # elif x=='h':
        # print("high")
        # p.ChangeDutyCycle(75)
        # x='z'
     
    
    elif x=='e':
        GPIO.cleanup()
        print("GPIO Clean up")
        break
    
    else:
        print("<<<  wrong data  >>>")
        print("please enter the defined data to continue.....")
