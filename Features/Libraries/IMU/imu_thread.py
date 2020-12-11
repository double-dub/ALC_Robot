import serial
import threading

# Arduino Communication IMU
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=None)
angle = 0.0


def runnit():
    global angle
    print("Press i to print angle from threaded function:")
    print("Press o to exit")

    flag = True

    while flag:
        value = input()
        if value == 'i':
            print(angle)
        elif value == 'o':
            print("bye")
            return


def imu_thread():
    global angle

    while True:
        data = ser.readline()
        decoded_bytes = data.decode("utf-8")
        if data:
            angle = float(decoded_bytes)


th = threading.Thread(target=imu_thread)
th.start()
runnit()
