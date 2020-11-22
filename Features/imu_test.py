import serial
import time

# Arduino Communication IMU
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=None)

angle = 0.0


def imu_read():
    s = []

    imu_halt()

    while ser.in_waiting != 0:
        data = ser.readline()
        decoded_bytes = data.decode("utf-8")
        if data:
            s.append(float(decoded_bytes))

    imu_start()

    try:
        angle = s[-1]
        return angle
    except ValueError:
        angle = angle


def imu_start():
    ser.write(b'g')


def imu_halt():
    ser.write(b'h')


imu_start()
time.sleep(0.01)
angle = imu_read()
print(angle)
