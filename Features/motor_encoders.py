import time
import serial
from motors import *

# Arduino Communication
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 19200
ser = serial.Serial(SERIAL_PORT,BAUD_RATE)

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
            print(s)
            if (enc1 >= 3592) :
               # stop()
                ser.write("Reset".encode("utf-8"))
                time.sleep(.1)
                print("what?")
                enc1 = s[0]
                enc2 = s[1]
                stop()
                ser.write("Reset".encode("utf-8"))
                print(decoded_bytes)
                break
                #ser.write("Reset".encode("utf-8"))
        if len(s) == 2:
            s = []

while True:
    value = input()
    if value == 'w':
        setspeed(1300)
        print("Forward")
        forward()
        print("\n")
        encoder_read()
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

