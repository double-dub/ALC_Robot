import serial

# Arduino Communication IMU
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE,timeout = None)

def imu_read():
  s = []
  count = 0
  while True:
    data = ser.readline()
    decoded_bytes = data.decode("utf-8")
    if data:
        s.append(decoded_bytes)
    if len(s) == 1:
      angle = s[0]
      print(angle)
    if len(s) == 1:
      s = []
    break
