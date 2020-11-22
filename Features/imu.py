import serial
import time

# Arduino Communication IMU
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE,timeout = None)

def imu_read():
    s = []
    count = 0
    while ser.in_waiting != 0:
        data = ser.readline()
        decoded_bytes = data.decode("utf-8")
        if data:
            s.append(float(decoded_bytes))
            angle = s[0]
            #print(s)
    last = s[-1]
    return last

def halt_imu():
    ser.write(b'h')

def start_imu():
    ser.write(b'g')

def get_angle():
    start_imu()
    time.sleep(.01)
    halt_imu()
    return imu_read()
print(get_angle())
