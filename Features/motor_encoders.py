from __future__ import division
import Adafruit_PCA9685
import RPi.GPIO as gpio
import time
import serial

# Initialize PCA9685 Board
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=0)

#initiate gpio pins & ignore warnings
gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)

#Inverse direction pin (low=forward, high=backward)
inv = 31
gpio.setup(inv,gpio.OUT)

# RIGHT WHEEL
#motor A initialize
#in1 and in2
m1_in1 = 29
m1_in2 = 23
gpio.setup(m1_in1,gpio.OUT)
gpio.setup(m1_in2,gpio.OUT)

# LEFT WHEEL
#motor B initialize
#in1 and in2
m2_in1 = 19
m2_in2 = 21
gpio.setup(m2_in1,gpio.OUT)
gpio.setup(m2_in2,gpio.OUT)

# PWM pins
pwm1 = 9
pwm2 = 8
pwm.set_pwm(pwm1,0,0)
pwm.set_pwm(pwm2,0,0)

# Arduino Communication
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT,BAUD_RATE)





'''
basic sudo code for driving straight with 
undefined distance

initialize error
error = 0

kp will have to be tested/adjusted
kp = .2

encoders to start at 0
enc1 = 0
enc2 = 0


-----MAIN LOOP-----

set motors pwm as a variable to change
with respect to encoder differences/values
will use "slave" and "master" to reference 
each motors
pwm1_master = 0
pwm2_slave = 0

set error variable
error = enc1 - enc2

change "slave" power. only slave motor
will change to catch up to master
pwm2_slave += error / kp

reset encoder every loop to start fresh after every reading:
enc1 = 0
enc2 = 0

sleep function to reset properly. 
without this sleep function, loop will
lose accuracy
time.sleep(some amount of time in seconds)

'''

'''
SUDO CODE FOR DRIVING STRAIGHT FOR A SET DISTANCE

DEFINING FUNCTIONS(tenths of an inch, master pwm power)
    tickGoal = (42 * tenths of an inch) / 10)

    totalticks = 0

    error = 0

    kp = 5
    
    leftencoder = 0
    rightencoder = 0

    WHILE LOOP
    leftmotor_pwm = master power
    rightmotor_pwm = slave power
    
    error = leftencoder - rightencoder
    
    reset encoder values
    leftencoder = 0
    rightencoder = 0

    sleep function to let loop reset properly
    time.sleep(time in seconds)

    totalTicks += leftencoder

Stop loop once encoders counted up the correct number of encoder ticks
leftmotor_pwm = 0
rightmotor_pwm = 0


EXAMPLE OF CALLING FUNCTION

FUNCTION(distance in inches, speed in which to travel PWM value)
time.sleep(let stuff reset properly)

'''




def encoder_pid():
    p = []
    kp = .2
    while True:
        data = ser.readline()
        decoded_bytes = int(data[0:len(data)-2].decode("utf-8"))
        if data:
            p.append(decoded_bytes)
        if len(p) == 2:
            speed = 1000
            slave_pwm = speed
            pwm.set_pwm(pwm1,0,1000)
            enc1 = p[0]
            enc2 = p[1]
            enc = [[],[]]
            enc[0].append(enc1)
            enc[1].append(enc2)
            print(enc)
            error = enc1 - enc2
            slave_pwm += round(error / kp)
            pwm.set_pwm(pwm2,0,slave_pwm)
        if len(p) == 2:
            p = []

def encoder_read():
    s = []
    while True:
        data = ser.readline()
        decoded_bytes = int(data[0:len(data)-2].decode("utf-8"))
        if data:
            s.append(decoded_bytes)
        if len(s) == 2:
            enc1 = s[0]
            enc2 = s[1]
            print(enc1)
            print(enc2)
            if enc1 > 3592 or enc2 > 3592:
                stop()
                print("Stop")
                break
        if len(s) == 2:
            print(s)
            s = []

def backward():
    #motor A
    gpio.output(m1_in1,gpio.LOW)
    gpio.output(m1_in2,gpio.HIGH)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.HIGH)

def forward():
    #motor A
    gpio.output(m1_in1,gpio.LOW)
    gpio.output(m1_in2,gpio.HIGH)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.LOW)

def stop():
    #motor A
    gpio.output(m1_in1,gpio.LOW)
    gpio.output(m1_in2,gpio.LOW)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.LOW)

def left():
    #motor A
    gpio.output(m1_in1,gpio.HIGH)
    gpio.output(m1_in2,gpio.LOW)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.HIGH)

def right():
    #motor A
    gpio.output(m1_in1,gpio.HIGH)
    gpio.output(m1_in2,gpio.LOW)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.LOW)

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
        #encoder_read()
        encoder_pid()
    elif value == 's':
        backward()
        print("Backward")
        print("\n")
        encoder_pid()
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

