from motors import *
from path import enc_res
from imu import *
from encoders import *
from encoderthread import *
import time
import serial
import struct
import smbus
import sys
import pretty_errors


def one_Lap():
    enc_res()
    forward()
           

def spincw_180():
    #spin clockwise 180 degrees from charging dock 

#long straight aways
ref_angle = 0
def ref_angle():
    global angle
    ref_angle = angle
    global ref_angle

def fwrd_180():
    global encoder1
    global ref_angle
    global angle
    #angle180 = ref_angle + 180
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<13887):
        forward()
        if ((angle180+.5) < angle):
            slight_right()
        elif ((angle180+.5) > angle):
            slight_left()
        elif ((angle180+.5) <= angle <=(angle180+ .5)):
            forward()
    stop()
    print(encoder1)

def fwrd_short():
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<6943):
        forward()
    stop()
    print(encoder1)



def pid():
    p = []
    kp = .5
    time.sleep(1)
    setspeedm1(1000)
    setspeedm2(1000)
    forward()
    while True:
        data = ser.readline()
        decoded_bytes = data[0:len(data)-2].decode("utf-8")
        if data:
            p.append(int(decoded_bytes))
        if len(p) == 2:
            speed = 1000
            slave_pwm = speed
            setspeedm1(1000)
            enc = [[],[]]
            enc[0].append(p[0])
            enc[1].append(p[1])
            print(enc)
            error = p[0] - p[1]
            print("Error: "+str(error))
            slave_pwm += round(error /  kp)%100
            setspeedm2(slave_pwm) 
            if (enc[0][0] >= 10000):
                stop()
                encf.e1 = enc[0][0]
                encf.e2 = enc[1][0]
                p = [[0],[0]]
        if decoded_bytes  == '':
            break
        if len(p) == 2:
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
    speed = 3000
    setspeedm1(speed)
    setspeedm2(speed)
    forward()
    while True:
        speed -= 2
        setspeedm1(speed)
        setspeedm2(speed)
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
            if (enc1 >= 10000):
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









# Start of IMU Stuff

vector_counter = 0 
the_2dlist = []

def imu_turn90():
    global vector_counter
    global the_2dlist
    speed = 1250
    cur_angle = imu_read()
    while cur_angle == "failed":
        cur_angle = imu_read()

    goal_angle = cur_angle + 84.5
    print("Start angle: " + str(cur_angle) + "\n")
    print("Goal angle: " + str(goal_angle) + "\n")
    while cur_angle < goal_angle:
        # turn-right
        cur_angle = imu_read()
        while cur_angle == "failed":
            cur_angle = imu_read()
        setspeed(speed)
        right()
        if (cur_angle < goal_angle*0.75):
            setspeed(speed)
    stop()
    print("Final angle : " + str(cur_angle) + "\n")
    print("Exited loop\n")
    time.sleep(1)
    reading()
    ret_angle = imu_read()
    the_2dlist.append([1.0, ret_angle])
    vector_counter = vector_counter + 1
    print(vector_counter)
    print(the_2dlist)    




vector_list = [
    [1.0, 0.0],
    [1.2, 45.0],
    [1.5, 135]
]


def calc_resultant(array2d):
    x_component = 0.0
    y_component = 0.0
    index = 0

    for _ in array2d:
        x_component += (array2d[index][0] * math.cos(math.radians(array2d[index][1])))
        y_component += (array2d[index][0] * math.sin(math.radians(array2d[index][1])))
        index += 1

    num = math.pow(x_component, 2) + math.pow(y_component, 2)
    resultant_mag = math.sqrt(num)
    resultant_deg = math.atan(y_component/x_component)

    return [round(resultant_mag, 3), round(math.degrees(resultant_deg), 3)]
		#res_vector = calc_resultant(vector_list)
		#print(res_vector)
	
#res_vector = calc_resultant(vector_list)
#print(res_vector)


def snakepath():
    encf()
    turn90L()
    encf2()
    turn90L()
    encf()
    turn90R()
    encf2()
    turn90R()
    encf()


enc_res()
while True:
    value = input()
    if value == 'w':
        print("Forward")
        print("\n")
        setspeedm1(1200)
        setspeedm2(1200)
        forward()
    elif value == 'pid':
        print("PID program \n")
        pid()
    elif value == 'fwrd':
        fwrd()
    elif value == 'encf':
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
        setspeedm1(1300)
        setspeedm2(1300)
        right()
        print("\n")
    elif value == 'a':
        print("Left")
        setspeedm1(1300)
        setspeedm2(1300) 
        left()
        # turn90L()
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
    elif value == 'q':
        print("Stop")
        stop()
        print("\n")
    elif value == 'shit':
        imu_turn90()
    elif value == 'read':
        reading()
    elif value == 'vector':
        # calc_resultant(vector_list)
        res_vector = calc_resultant(the_2dlist)
        print(res_vector)
    elif value == ' ':
        break
    else:
        print("<<< Invalid Key Entry >>>")
        print("\n")
