#import tkinter, time, pyfirmata, sys
#import serial
#from pyfirmata import Arduino, SERVO, PWM, INPUT, OUTPUT
from time import sleep
#from pyfirmata import Arduino, util
from adafruit_servokit import ServoKit
kit = ServoKit(channels=16)

'''
Often the range an individual servo recognises varies a bit from other servos.
If the servo didn't sweep the full expected range,
then try adjusting the minimum and maximum pulse widths using
set_pulse_width_range(min_pulse, max_pulse).

To set the pulse width range to a minimum of 1000 and a maximum of 2000:
kit.servo[0].set_pulse_width_range(1000, 2000)
'''
'''
# Setting up the Arduino board
port = 'COM7'
strComBaud = 9600
board = Arduino(port)
'''



#BTN = board.get_pin('d:8:i')


#arduino sync time
sleep(1)

# Set pin modes as SERVO
#pin8 = 8
pin9 = 9
pin10 = 10
pin11 = 11

'''
#board.digital[pin8].mode = INPUT
board.digital[8].read()
#board.digital[pin8].mode = SERVO
board.digital[pin9].mode = SERVO
board.digital[pin10].mode = SERVO
board.digital[pin11].mode = SERVO
'''

def setServoAngle1(pin9, angle1):
  #board.digital[pin9].write(angle1)
  kit.servo[pin9].angle = angle1
  sleep(.010)

def setServoAngle2(pin10, angle2):
  #board.digital[pin10].write(angle2)
  kit.servo[pin10].angle = angle2
  sleep(.010)

def setServoAngle3(pin11, angle3):
  #board.digital[pin11].write(angle3)
  kit.servo[pin11].angle = angle3
  sleep(.010)

#setServoAngle1(pin9, 180)
#setServoAngle2(pin10, 180)

def funct1():
  i = 180
  j = 180
  while (i<=180):
    setServoAngle1(pin9, i)
  
    if (70<=j<=180):
      setServoAngle2(pin10, j)
      setServoAngle1(pin9, i)
      j-=1
      i-=1
      print("i=:",i)
      print("j=:",j)
    else:
      print("i=:",i)
      print("j=:",j)
      i -=1
    if i ==20:
      break
  sleep(2)

def holdball():
  setServoAngle3(pin11, 0)
  sleep(2)
  setServoAngle3(pin11, 70)
  sleep(2)
  

def funct2():
  i = 20
  j = 69
  while (i<=180):
    setServoAngle1(pin9, i)
    
    if (j<=180):
      setServoAngle2(pin10, j)
      setServoAngle1(pin9, i)
      j+=1
      i+=1
      print("ib=:",i)
      print("jb=:",j)
    else:
      print("ix=:",i)
      print("jx=:",j)
      i +=1
    if i ==0:
      break
  sleep(1)

#setServoAngle3(pin11, 0)
#sleep(1)
funct1()
holdball()
funct2()
'''
while True:
  thread = util.Iterator(board)
  thread.start()
  if BTN.read() == True:
    print(BTN.read())
  else:
    print(BTN.read())
'''
