# Simple dem  of of the PCA9685 PWM sdge for HL-L2320Dtoner cartridge for HL-L2320Dtoner cartridge for HL-L2320Dtoner cartridge for HL-L2320Dtoner cartridge for HL-L2320Dtoner cartridge for HL-L2320Dervo/LED controller library.
# box Simple demo of of the PCA9685 PWM servo/LED controller library.
# Simple demo of of the PCA9685 PWM servo/LED controller library.

# This will move channel 0 from min to max position repeatedly.
#to turn off servo use pwm.set_pwm(channel, 4096, 0)
#turn on servo fully pwm.set_pwm(channel, 0, 4096)
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685
import RPi.GPIO as gpio

#initiate gpio pins & ignore warnings
gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)



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
    #Joint one at resting
    pwm.set_pwm(10, 0, 650)
    pwm.set_pwm(11, 0, 650)
    #Joint two at resting
    pwm.set_pwm(12, 0, 650)
    #Joint three gripper at resting
    pwm.set_pwm(13, 0, 375)
    pwm.set_pwm(14, 0, 300)
    time.sleep(.5)
    dig_stop(14)
    stop_servo(10)

    stop_servo(11)
    stop_servo(12)
def extend_ball():
    #pwm.set_pwm(1, 0, servo_min)
    #pwm.set_pwm(2, 0, servo_min)
    i = 650
    j = 650
    while (160<=i<=650):
        
  
        if (425<=j<=650):
            pwm.set_pwm(11, 0, i)
            pwm.set_pwm(12, 0, j)
            j-=1
            i-=1
            print("i=:",i)
            print("j=:",j)
            time.sleep(.002)
        else:
            #whichever joint is still moving after one of them stops
            pwm.set_pwm(11, 0, i)
            print("i=:",i)
            print("j=:",j)
            i -=1
            time.sleep(.002)
        if i== 160:
            print("test")
            break

def extend_cup():
    #pwm.set_pwm(1, 0, servo_min)
    #pwm.set_pwm(2, 0, servo_min)
    i = 650
    j = 650
    pwm.set_pwm(13, 0, 650)
    while (200<=i<=650):
        
    
        if (450<=j<=650):
            pwm.set_pwm(11, 0, i)
            pwm.set_pwm(12, 0, j)
            j-=1
            i-=1
            print("i=:",i)
            print("j=:",j)
            time.sleep(.006)
        else:
            #whichever joint is still moving after one of them stops
            pwm.set_pwm(11, 0, i)
            print("i=:",i)
            print("j=:",j)
            i -=1
            time.sleep(.01)
        if i== 200:
            #stop_servo(11)
            #stop_servo(12)
            break



def grab_ball():
    #sets servo angle of gripper to squeeze object
    time.sleep(.5)
    pwm.set_pwm(14, 0, 300)
    time.sleep(.5)
    pwm.set_pwm(14, 0, 500)
    time.sleep(.25)
    pwm.set_pwm(14, 4095, 0)
    time.sleep(1)

def grab_cup():
    #sets servo angle of gripper to squeeze object
    time.sleep(.5)
    pwm.set_pwm(14, 0, 300)
    time.sleep(.5)
    pwm.set_pwm(14, 0, 585)
    time.sleep(1)
    pwm.set_pwm(14, 4095, 0)
    time.sleep(1)

def retrieve_ball():
    #work backwards from grab positions
    i = 150
    j = 450
    while (150<=i<=650):
        
  
        if (450<=j<=650):           
            pwm.set_pwm(11, 0, i)
            pwm.set_pwm(12, 0, j)
            j+=1
            i+=1
            print("i=:",i)
            print("j=:",j)
            time.sleep(.006)
        else:
            #whichever joint is still moving after one of them stops
            pwm.set_pwm(11, 0, i)
            print("i=:",i)
            print("j=:",j)
            i +=1
            time.sleep(.006)
        if i == 650:
            print("retrieve")
            break

def retrieve_cup():
    #work backwards from grab positions
    i = 150
    j = 350
    pwm.set_pwm(13, 0, 650)
    while (150<=i<=650):
        
  
        if (350<=j<=650):           
            pwm.set_pwm(11, 0, i)
            pwm.set_pwm(12, 0, j)
            j+=1
            i+=1
            print("i=:",i)
            print("j=:",j)
            time.sleep(.004)
        else:
            #whichever joint is still moving after one of them stops
            pwm.set_pwm(11, 0, i)
            print("i=:",i)
            print("j=:",j)
            i +=1
            time.sleep(.004)
        if i == 650:
            print("retrieve")
            stop_servo(11)
            stop_servo(12)
            break



def spindrop():
    pwm.set_pwm(11, 0, 650)
    pwm.set_pwm(12, 0, 650)
    pwm.set_pwm(10, 0, 650)
    pwm.set_pwm(10, 0, 125)
    time.sleep(1)
    i = 650
    j = 650
    while (400<=i<=650):
        
  
        if (550<=j<=650):
            pwm.set_pwm(11, 0, i)
            pwm.set_pwm(12, 0, j)
            j-=1
            i-=1
            print("i=:",i)
            print("j=:",j)
            time.sleep(.002)
        else:
            #whichever joint is still moving after one of them stops
            pwm.set_pwm(11, 0, i)
            print("i=:",i)
            print("j=:",j)
            i -=1
            time.sleep(.002)
        if i == 150:
            print("test")
            break
    
    print("release")
    pwm.set_pwm(14, 0, 250)
    time.sleep(1)
    #pwm.set_pwm(10, 4096, 0)
    stop_servo(10)
    stop_servo(14)

def stop_servo(ch):
    pwm.set_pwm(ch, 4096, 0)
    #time.sleep(.25)
def dig_stop(ch):
    pwm.set_pwm(ch, 4095, 0)
    time.sleep(.5)
