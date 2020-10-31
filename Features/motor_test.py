import RPi.GPIO as gpio
import time

#initiate gpio pins & ignore warnings
gpio.setmode(gpio.BCM)
gpio.setwarnings(False)

#motor A initialize
#enable pins
gpio.setup(2,gpio.OUT)
gpio.setup(3,gpio.OUT)
#in1 and in2
gpio.setup(27,gpio.OUT)
gpio.setup(22,gpio.OUT)

#motor B initialize
#enable pins
gpio.setup(4,gpio.OUT)
gpio.setup(17,gpio.OUT)
#in1 and in2
gpio.setup(10,gpio.OUT)
gpio.setup(9,gpio.OUT)

#activate motor A
gpio.output(2,True)
gpio.output(3,True)

#activate motor B
gpio.output(4,True)
gpio.output(17,True)

#custom motor speeds using PWM values
p1 = gpio.PWM(2,1000)
p2 = gpio.PWM(17,1000)
p1.start(50)
p2.start(50)
temp1 = 1

def backward():
    #motor A
    gpio.output(27,gpio.HIGH)
    gpio.output(22,gpio.LOW)
    #motor B
    gpio.output(10,gpio.HIGH)
    gpio.output(9,gpio.LOW)
    temp1=1
    
def forward():
    #motor A
    gpio.output(27,gpio.LOW)
    gpio.output(22,gpio.HIGH)
    #motor B
    gpio.output(10,gpio.LOW)
    gpio.output(9,gpio.HIGH)
    temp1=0
    
def stop():
    #motor A
    gpio.output(27,gpio.LOW)
    gpio.output(22,gpio.LOW)
    #motor B
    gpio.output(10,gpio.LOW)
    gpio.output(9,gpio.LOW)

def left():
    p1.ChangeDutyCycle(100)
    p2.ChangeDutyCycle(100)
    #motor A
    gpio.output(27,gpio.LOW)
    gpio.output(22,gpio.HIGH)
    #motor B
    gpio.output(10,gpio.HIGH)
    gpio.output(9,gpio.LOW)

def right():
    p1.ChangeDutyCycle(100)
    p2.ChangeDutyCycle(100)
    #motor A
    gpio.output(27,gpio.HIGH)
    gpio.output(22,gpio.LOW)
    #motor B
    gpio.output(10,gpio.LOW)
    gpio.output(9,gpio.HIGH)

print("\n")
print("Motor Tester...Press spacebar to quit\n")
print("Use the following 'wasd' keys to control the robot: \n")
print("Forward - w  Backward - s  Right - d  Left - a  Stop - q")
print("\n")

while True:
    value = input()
    if value == 'w':
        print("Forward")
        forward()
        print("\n")
    elif value == 's':
        backward()
        print("Backward")
        print("\n")
    elif value == 'd':
        print("Right")
        right()
        print("\n")
    elif value == 'a':
        print("Left")
        left()
        print("\n")
    elif value == 'q':
        print("Stop")
        stop()
        print("\n")
    elif value == ' ':
        break
    else:
        print("<<< Invalid Key Entry >>>")
        print("\n")
