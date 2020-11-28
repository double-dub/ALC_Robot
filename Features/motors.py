from __future__ import division
import Adafruit_PCA9685
import RPi.GPIO as gpio
import time

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
m1_in1 = 23
m1_in2 = 29
gpio.setup(m1_in1,gpio.OUT)
gpio.setup(m1_in2,gpio.OUT)

# LEFT WHEEL
#motor B initialize
#in1 and in2
m2_in1 = 21
m2_in2 = 19
gpio.setup(m2_in1,gpio.OUT)
gpio.setup(m2_in2,gpio.OUT)

# PWM pins
pwm1 = 8
pwm2 = 9

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

def slight_right():
    #motor A
    gpio.output(m1_in1,gpio.LOW)
    gpio.output(m1_in2,gpio.HIGH)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.LOW)
    pwm.set_pwm(pwm2,0,3000)
    pwm.sey_pwm(pwm1,0,2800)

def slight_left():
    #motor A
    gpio.output(m1_in1,gpio.LOW)
    gpio.output(m1_in2,gpio.HIGH)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.LOW)
    pwm.set_pwm(pwm2,0,2800)
    pwm.sey_pwm(pwm1,0,3000)



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

def aleft():
    #motor A
    gpio.output(m1_in1,gpio.HIGH)
    gpio.output(m1_in2,gpio.LOW)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.HIGH)
    
def aright():
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



def setspeedm1(speed):
    pwm.set_pwm(pwm2,0,speed)

def setspeedm2(speed):
    pwm.set_pwm(pwm1,0,speed)
