from threading import Thread
import serial
import time
import pretty_errors


# Arduino Communication IMU
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=None)

angle = 0.0


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

        imu_start()
        angle = s[-1]

    except Exception:
        angle = angle

    return angle


def imu_start():
    ser.write(b'g')


def imu_halt():
    ser.write(b'h')


imu_start()

for x in range(10):
    time.sleep(0.01)
    angle = imu_read()
    print(angle)
