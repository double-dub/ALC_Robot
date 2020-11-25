import serial
import time
import math

# Arduino Communication IMU
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE,timeout = None)
angle = 0.0


def imu_read_test():
    ser.write(b'g')
    time.sleep(.01)
    data = ser.readline()
    decoded_bytes = data.decode("utf-8")
    s = []
    s.append(float(decoded_bytes))
    print("decoded_bytes\n", decoded_bytes)
    angle_0 = s[-1]
    #left() 
    print("angle_0")
    print(angle_0)
    while ser.in_waiting != 0:
        ser.write(b'g')
        time.sleep(.01)
        #ser.write(b'h')
        data = ser.readline()
        decoded_bytes = data.decode("utf-8")
        print("decoded_bytes\n", decoded_bytes)
        s.append(float(decoded_bytes))
        new_angle = s[-1]
        delta_angle=abs(new_angle-angle_0)
        #left()
        #time.sleep(1)
        #stop()
        print("delta")
        print(delta_angle)
        ser.write(b'h')
        '''
        if data:
            s.append(float(decoded_bytes))
            angle = s[0]
            #print(s) 
        '''
    last = s[-1]
    return last
    print("last")
    print(last)


def imu_read():
    global angle
    s = []

    imu_halt()

    try:
        while ser.in_waiting != 0:
            data = ser.readline()
            decoded_bytes = data.decode("utf-8")
            if data:
               s.append(float(decoded_bytes))

        angle = s[-1]

    except Exception:
        angle = "failed"

    imu_start()

    return angle


def imu_start():
    ser.write(b'g')


def imu_halt():
    ser.write(b'h')


def readings():
    global angle
    imu_start()

    for x in range(10):
        time.sleep(0.01)
        angle = imu_read()
        print(angle)


def get_angle():
    start_imu()
    time.sleep(.01)
    halt_imu()
    return imu_read()

def reading():
    global angle
    s = []

    imu_halt()

    try:
        while ser.in_waiting != 0:
            data = ser.readline()
            decoded_bytes = data.decode("utf-8")
            if data:
               s.append(float(decoded_bytes))

        angle = s[-1]

    except Exception:
        angle = angle

    imu_start()

    print(angle)


