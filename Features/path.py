from motors import *
from imu import *
import time
import serial
import struct
import smbus
import sys
import pretty_errors

# Arduino Communication
SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT,BAUD_RATE,timeout=1)

#i2c bus for the  power shield battery %
bus = smbus.SMBus(1)

def readVoltage(bus):

     address = 0x36
     read = bus.read_word_data(address, 2)
     swapped = struct.unpack("<H", struct.pack(">H", read))[0]
     voltage = swapped * 1.25 /1000/16
     return voltage


def readCapacity(bus):

     address = 0x36
     read = bus.read_word_data(address, 4)
     swapped = struct.unpack("<H", struct.pack(">H", read))[0]
     capacity = swapped/256
     return capacity

def encoder_pid(distance):
    p = []
    kp = .4
    time.sleep(1)
    data = ser.readline()
    decoded_bytes = int(data[0:len(data)-2].decode("utf-8"))
    
    while (p[0]<distance):
        forward()
        data = ser.readline()
        decoded_bytes = int(data[0:len(data)-2].decode("utf-8"))
        if data:
            p.append(decoded_bytes)
        
        if (p[0] > 3952):
            #ser.write(b"hi fren")
            stop()
            #enc_res()
        if len(p) == 2:
            speed = 1000
            slave_pwm = speed
            pwm.set_pwm(pwm1,0,1000)
            enc = [[],[]]
            enc[0].append(p[0])
            enc[1].append(p[1])
            print(enc)
            error = p[0] - p[1]
            slave_pwm += round(error / kp)
            pwm.set_pwm(pwm2,0,slave_pwm)
        p = []


def encoder_reads():
    s = []
    while True:
        data = ser.readline()
        decoded_bytes = data[0:len(data)-2].decode("utf-8")
        if data:
            s.append(int(decoded_bytes))
        if len(s) == 2:
            enc1 = s[0]
            enc2 = s[1]
            enc = [[],[]]
            enc[0].append(enc1)
            enc[1].append(enc2)
            print(enc)
        if len(s) == 2:
            s = []


def encf():
    s = []
    forward()
    while True:
        data = ser.readline()
        decoded_bytes = data[0:len(data)-2].decode("utf-8")
        if data:
            s.append(int(decoded_bytes))
        if len(s) == 2:
            enc1 = s[0]
            enc2 = s[1]
            enc = [[],[]]
            enc[0].append(enc1)
            enc[1].append(enc2)
            print(enc)
            if (enc1 >= 3952):
                stop()
                encf.e1 = enc1
                encf.e2 = enc2
                s = [[0],[0]]
        if decoded_bytes  == '':
            break
        if len(s) == 2:
            s = []

def last_val():
    print("last values")
    print(last_val[0]+' '+last_val[1])

def turn90L():
    s = []
    left()
    while True:
        data = ser.readline()
        ser.flush()
        decoded_bytes = data[0:len(data)-2].decode("utf-8")
        if data:
            s.append(int(decoded_bytes))
        if len(s) == 2:
            enc1 = s[0]
            enc2 = s[1]
            enc = [[],[]]
            enc[0].append(enc1)
            enc[1].append(enc2)
            print(enc)
            if (enc1 <= -1796) or (enc2 >= 1796) :
                stop()
        if decoded_bytes == '':
            break
        if len(s) == 2:
            #print(s)
            s = []
 
def turn90R():
    s = []
    while True:
        data = ser.readline()
        ser.flush()
        decoded_bytes = int(data[0:len(data)-2].decode("utf-8"))
        if data:
            s.append(decoded_bytes)
        if len(s) == 2:
            enc1 = s[0]
            enc2 = s[1]
            print(enc1)
            print(enc2)
            if (enc1 >= 1796) or (enc2 <= -1796) :
                stop()
                enc1 = s[0]
                enc2 = s[1]
                stop()
                break
        if len(s) == 2:
            print(s)
            s = []
 
def readenc():
    #ser.write("b'0\r\n'")
    #ser.write(b" ")
    #ser.flush()
    #print("Encoder Reset")
    l = []
    data = ser.readline()
    decoded_bytes = data[0:len(data)-2].decode("utf-8")
    if data:
        l.append(decoded_bytes)
    if len(l) == 2:
        print(decoded_bytes)

def enc_res():
    ser.write(b"0")
    print("Reset")

def rpath():
    #Set Speed
    setspeed(1300)  
    #Forward
    encf()
    #Reset
    enc_res()
    time.sleep(1)    
    #Left Turn
    turn90L()
    #Reset
    enc_res()
    time.sleep(1)
    #Forward
    encf()
    #Reset
    enc_res()
    time.sleep(1)
    #Left Turn
    turn90L()
    #Reset
    enc_res()
    time.sleep(1)
    #Forward
    encf()
    #Reset
    enc_res()
    time.sleep(1)
    #Left Turn
    turn90L()
    #Reset
    enc_res()
    time.sleep(1)


def imu_turn90():
    cur_angle = imu_read()
    while cur_angle == "failed":
        cur_angle = imu_read()

    goal_angle = cur_angle + 90
    while cur_angle < goal_angle:
        # turn-right
        cur_angle = imu_read()
        while cur_angle == "failed":
            cur_angle = imu_read()
        setspeed(1250)
        right()
    stop()
    print("Exited loop\n")


print("\n")
print("Motor Tester...Press spacebar to quit\n")
print("Use the following 'wasd' keys to control the robot: \n")
print("Forward - w  Backward - s  Right - d  Left - a  Stop - q Battery% - b")
print("\n")

while True:
    value = input()
    if value == 'w':
        print("Forward")
        print("\n")
        setspeed(1200)
        forward()
        encoder_read()
    elif value == 'encf':
        setspeed(1000)
        encf()
        print(encf.e1)
        print(encf.e2)
    elif value == 'ww':
        print("\n")
        encoder_reads()
    elif value == 's':
        backward()
        print("Backward")
        print("\n")
    elif value == 'd':
        print("Right")
        right()
        turn90R()
        print("\n")
    elif value == 'a':
        print("Left")
        left()
        turn90L()
        print("\n")
    elif value == 'r':
        readenc()
        print("\n")
    elif value == 't':
        enc_res()
        print("\n")
    elif value == 'p':
        encoder_pid()
    elif value == '1':
        encoder_read()
    elif value == 'b':
        print("Voltage:%5.2fV" % readVoltage(bus))
        print("Battery:%5i%%" % readCapacity(bus))
    elif value == 'q':
        print("Stop")
        stop()
        print("\n")
    elif value == 'path':
        print("Path")
        rpath()
    elif value == 'shit':
        imu_turn90()
    elif value == ' ':
        break
    else:
        print("<<< Invalid Key Entry >>>")
        print("\n")
