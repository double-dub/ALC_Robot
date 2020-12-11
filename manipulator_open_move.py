import tkinter, time, pyfirmata, sys
import serial
from pyfirmata import Arduino, SERVO, PWM, INPUT, OUTPUT
from time import sleep
from pyfirmata import Arduino, util


# Setting up the Arduino board
port = 'COM4'
strComBaud = 9600
board = Arduino(port)

#used for getting state
#BTN = board.get_pin('d:8:i')


#arduino sync time
#sleep(1)

# Set pin modes as SERVO
#pin8 = 8
pin9 = 9
pin10 = 10
pin11 = 11
pin12 = 12
pin13 = 13

#board.digital[pin8].mode = INPUT
#board.digital[8].read()

board.digital[pin9].mode = SERVO
board.digital[pin10].mode = SERVO
board.digital[pin11].mode = SERVO
board.digital[pin12].mode = SERVO
board.digital[pin13].mode = SERVO



def setServoAngle1(pin9, angle1):
  board.digital[pin9].write(angle1)
  sleep(0.00001)

def setServoAngle2(pin10, angle2):
  board.digital[pin10].write(angle2)
  sleep(0.00001)

def setServoAngle3(pin11, angle3):
  board.digital[pin11].write(angle3)
  sleep(.00001)

def setServoAngle4(pin12, angle4):
  board.digital[pin11].write(angle4)
  sleep(.00001)

def setServoAngle5(pin13, angle5):
  board.digital[pin11].write(angle5)
  sleep(.00001)


'''
thread = util.Iterator(board)
thread.start()
x = BTN.read()
'''
def funct1():
  setServoAngle1(pin9, 90)
  setServoAngle4(pin12, 90)
  i = 180
  j = 180
  while (10<=i<=180):
    setServoAngle2(pin10, i)
  
    if (80<=j<=180):
      setServoAngle3(pin11, j)
      setServoAngle2(pin10, i)
      j-=1
      i-=1
      print("i=:",i)
      print("j=:",j)
    else:
      print("i=:",i)
      print("j=:",j)
      i -=1
    if i ==0:
      break
  sleep(2)

def holdball():
  setServoAngle1(pin9, 90)
  setServoAngle4(pin12, 90)
  setServoAngle5(pin13, 0)
  sleep(1)
  setServoAngle5(pin13, 90)
  sleep(1)
  setServoAngle5(pin13, 0)
  sleep(1)

def funct2():
  setServoAngle1(pin9, 90)
  setServoAngle4(pin12, 90)
  i = 1
  j = 59
  while (i<=180):
    setServoAngle2(pin10, i)
    
    if (j<=180):
      setServoAngle3(pin11, j)
      setServoAngle2(pin10, i)
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


funct1()
#holdball()
#funct2()

'''
while True:
  #arduino iterator will check for high(3.3v w/ resistor) from
  #a gpio pin on jetson nano after robot is within
  #acceptable range within the object
  thread = util.Iterator(board)
  thread.start()
  #once pin is high or true, run manipulator
  if BTN.read() == True:
    funct1()
    holdball()
    funct2()
    #once manipulator done, write pin 7 as high and
    #send signal to Jetson nano, Jetsen will be looking for this signal
    #to turn pin8 signal off or False. 
    board.digital[7].write(1)
    #this pause will make sure arduino doesnt enter loop again prematurely
    #from sending the turnoff signal while pin8 is still turned on
    sleep(.5)

  if anoother pin high:
    drop ball
'''
