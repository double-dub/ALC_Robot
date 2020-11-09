<<<<<<< HEAD
# box Simple demo of of the PCA9685 PWM servo/LED controller library.
=======
# test Simple demo of of the PCA9685 PWM servo/LED controller library.
>>>>>>> 711fe21cc1f413c2e35a27246a115547763c6e7f
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
#pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=0)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

'''
print('Moving servo on channel 0, press Ctrl-C to quit...')
while True:
    # Move servo on channel O between extremes.
    pwm.set_pwm(0, 0, servo_min)
    time.sleep(1)
    pwm.set_pwm(0, 0, servo_max)
    time.sleep(1)
'''
def rest():
    #keeps base joint straight forward
    pwm.set_pwm(0, 0, servo_min)
    #Joint one at resting
    pwm.set_pwm(1, 0, servo_min)
    #Joint two at resting
    pwm.set_pwm(2, 0, servo_min)
    #Joint three gripper at resting
    pwm.set_pwm(3, 0, servo_min)

def extend():
    #pwm.set_pwm(1, 0, servo_min)
    #pwm.set_pwm(2, 0, servo_min)
    i = 180
    j = 180
    while (10<=i<=180):
        
  
        if (80<=j<=180):
            setServoAngle3(pin11, j)
            setServoAngle2(pin10, i)
            j-=1
            i-=1
            print("i=:",i)
            print("j=:",j)
            time.sleep(.25)
        else:
            #whichever joint is still moving after one of them stops
            pwm.set_pwm(2, 0, j)
            print("i=:",i)
            print("j=:",j)
            i -=1
            time.sleep(.25)
        if i ==0:
            break

def grab():
    #sets servo angle of gripper to squeeze object
    pwm.set_pwm(3, 0, #)

def retrieve():
    #work backwards from grab positions
    i = 180
    j = 180
    while (10<=i<=180):
        
  
        if (80<=j<=180):
            setServoAngle3(pin11, j)
            setServoAngle2(pin10, i)
            j-=1
            i-=1
            print("i=:",i)
            print("j=:",j)
            time.sleep(.25)
        else:
            #whichever joint is still moving after one of them stops
            pwm.set_pwm(2, 0, j)
            print("i=:",i)
            print("j=:",j)
            i -=1
            time.sleep(.25)
        if i ==0:
            break

