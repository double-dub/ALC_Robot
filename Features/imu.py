import serial

# Arduino Communication IMU
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout = None)

'''
# Thread function to read imu angles from arduino continuously
def imu_thread():
    global angle
    
    while True:
        data = ser.readline()
        decoded_bytes = data.decode("utf-8")
        if data:
            try:
                angle = float(decoded_bytes)
            except Exception:
                pass
'''
