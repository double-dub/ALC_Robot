# Simple demo of of the PCA9685 PWM servo/LED controller library.
# box Simple demo of of the PCA9685 PWM servo/LED controller library.
# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
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
#servo_min = 150  # Min pulse length out of 4096
#servo_max = 600  # Max pulse length out of 4096

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
    #pwm.set_pwm(0, 0, servo_min)
    #Joint one at resting
    pwm.set_pwm(11, 0, 200)
    #Joint two at resting
    pwm.set_pwm(12, 0, 150)
    #Joint three gripper at resting
    pwm.set_pwm(14, 0, 400)
    time.sleep(1)
    pwm.set_pwm(14, 4095, 0)

def extend():
    #pwm.set_pwm(1, 0, servo_min)
    #pwm.set_pwm(2, 0, servo_min)
    i = 150
    j = 150
    while (150<=i<=600):
        
  
        if (150<=j<=290):
            pwm.set_pwm(11, 0, i)
            pwm.set_pwm(12, 0, j)
            j+=1
            i+=1
            print("i=:",i)
            print("j=:",j)
            time.sleep(.009)
        else:
            #whichever joint is still moving after one of them stops
            pwm.set_pwm(11, 0, i)
            print("i=:",i)
            print("j=:",j)
            i +=1
            time.sleep(.009)
        if i == 600:
            print("test")
            break

def grab():
    #sets servo angle of gripper to squeeze object
    time.sleep(1)
    pwm.set_pwm(14, 0, 400)
    time.sleep(1)
    pwm.set_pwm(14, 0, 500)
    print("grab")
    time.sleep(1)
    pwm.set_pwm(14, 4095, 0)

def retrieve():
    #work backwards from grab positions
    i = 600
    j = 290
    while (150<=i<=600):
        
  
        if (150<=j<=290):           
            pwm.set_pwm(11, 0, i)
            pwm.set_pwm(12, 0, j)
            j-=1
            i-=1
            print("i=:",i)
            print("j=:",j)
            time.sleep(.009)
        else:
            #whichever joint is still moving after one of them stops
            pwm.set_pwm(11, 0, i)
            print("i=:",i)
            print("j=:",j)
            i -=1
            time.sleep(.009)
        if i == 150:
            print("retrieve")
            break

#thewse commands used in order
rest()
extend()
grab()
retrieve()

'''
while True:
    value = input()
    if value == 'w':
        print("Forward")
       
        #extend()
        
        pwm.set_pwm(11, 4095, 0)
        pwm.set_pwm(12, 4095, 0)
        pwm.set_pwm(14, 4095, 0)

        
        #pwm.set_pwm(14, 0, 150)
        time.sleep(2)
        #sends servo a low signal so that it relaxes the servo
        #pwm.set_pwm(14, 0, 400)
        time.sleep(2)
        #pwm.set_pwm(13, 4095, 0)
        print("\n")
    elif value == 's':
        #backward()
        pwm.set_pwm(14, 0, 150)
        time.sleep(2)
        pwm.set_pwm(14, 0, 600)
        time.sleep(2)

        print("Backward")
        print("\n")
        
    elif value == 'd':
        print("Right")
        #right()
        pwm.set_pwm(15, 0, 150)
        time.sleep(2)
        pwm.set_pwm(15, 0, 600)
        time.sleep(2)

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
    '''
