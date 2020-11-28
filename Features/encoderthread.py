from encoders import *
from imu import *
from motors import *
import threading
import pretty_errors
from time import sleep
from resultant_vector import *
from manipulator import *

angle = 0.0
encoder1 = 0
encoder2 = 0
error = 0.0
vector_list = []

def enc_res():
    serial_e.write(b"0")
    print("Reset")

def encoder_thread():
    global encoder1
    global encoder2
    elist = []
    while True:
        while serial_e != 0:
            data = serial_e.readline()
            decoded_bytes = data[0:len(data)-2].decode("utf-8")
            if data:
                elist.append(int(decoded_bytes))
            if len(elist) == 2:
                encoder1 = elist[0]
                encoder2 = elist[1]
            if len(elist) == 2:
                elist = []


def imu_thread():
    global angle

    while True:
        data = ser.readline()
        decoded_bytes = data.decode("utf-8")
        if data:
            angle = float(decoded_bytes)


def fwrd():
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<10000):
        setspeedm1(1400)
        setspeedm2(1400)
        forward()
    stop()
    print(encoder1)


def fwrd_short():
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<6943):
        setspeedm1(1400)
        setspeedm2(1400)
        forward()
    stop()
    print(encoder1)


def imu_turn90R():
    global angle
    global error
    cur_angle = angle 
    goal_angle = cur_angle + 86 - error
    
    print("Start angle: " + str(cur_angle) + "\n")
    print("Goal angle: " + str(goal_angle) + "\n")
    
    while cur_angle < goal_angle:
        cur_angle = angle
        setspeedm1(1100)
        setspeedm2(1100)
        right()
    
    stop()
    
    print("Final angle: " + str(cur_angle) + "\n")
    print("Exited loop\n")    


def imu_turn90L():
    global angle
    global error
    cur_angle = angle 
    goal_angle = cur_angle - 86 + error

    print("Start angle: " + str(cur_angle) + "\n")
    print("Goal angle: " + str(goal_angle) + "\n")
    
    while cur_angle > goal_angle:
        cur_angle = angle 
        setspeedm1(1100)
        setspeedm2(1100)
        left()

    stop()

    print("Final angle: " + str(cur_angle) + "\n")
    print("Exited loop\n")


def zoom(ticks):
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<ticks):
        setspeedm1(4095)
        setspeedm2(4095)
        forward()
    stop()
    print(encoder1)


def return_base():
    global angle
    res_mag, res_deg = calc_resultant(vector_list)
    if (res_deg - 180) < 0:
        res_deg = (res_deg - 180) % 360
    print(res_mag)
    print(res_deg)
    cur_angle = angle

    while True:
        if((cur_angle < (res_deg + 1)) and (cur_angle > (res_deg - 1))):
            stop()
            print(angle)
            zoom(res_mag)
            print(angle)
            break
        else:
            cur_angle = angle
            setspeedm1(1100)
            setspeedm2(1100)
            left()

    stop()
    # zoom(res_mag)
    print(angle)


imu_th = threading.Thread(target=imu_thread)
enc_th = threading.Thread(target=encoder_thread)
imu_th.start()
enc_th.start()


print("Enter 1 for encoder1 reading.")
print("Enter 2 for encoder2 reading.")
print("Enter 3 for imu reading.")
print("Enter 4 for all readings.")
print("Enter tr to turn 90-degrees right.\n")

while True:
    val = input()
    
    if val == '1':
        print("Encoder 1 Reading: " + str(encoder1) + "\n")
    elif val == '2':
        print("Encoder 2 Reading: " + str(encoder2) + "\n")
    elif val == '3':
        print("IMU Reading: " + str(angle) + "\n")
    elif val == '4':
        print("Enc1: " + str(encoder1) + "\nEnc2: " + str(encoder2) + "\nIMU: " + str(angle) + "\n")
    elif val == 'f':
        setspeedm1(1250)
        setspeedm2(1250)
        fwrd()
    elif val == 'r':
        enc_res()
    elif val == 'tr':
        imu_turn90R()
    elif val == 'tl':
        imu_turn90L()
    elif val == ' ':
        break
    elif val == 'path':
        fwrd()
        enc_avg = round((encoder1 + encoder2)/2)
        vector_list.append([enc_avg, angle])
        enc_res()
        sleep(1)
        imu_turn90R()
        sleep(1)
        fwrd_short()
        enc_avg = round((encoder1 + encoder2)/2)
        vector_list.append([enc_avg, angle])
        enc_res()
        sleep(1)
        imu_turn90R()
        sleep(1)
        fwrd()
        enc_avg = round((encoder1 + encoder2)/2)
        vector_list.append([enc_avg, angle])
        enc_res()
        sleep(1)
        imu_turn90L()
        sleep(1)
        fwrd_short()
        enc_avg = round((encoder1 + encoder2)/2)
        vector_list.append([enc_avg, angle])
        enc_res()
        sleep(1)
        imu_turn90L()
        sleep(1)
        fwrd()
        enc_avg = round((encoder1 + encoder2)/2)
        vector_list.append([enc_avg, angle])
        enc_res()
        print(vector_list)
        return_base() 
