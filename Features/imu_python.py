import serial

SERIAL_PORT = '/dev/tty0'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

def encoder_read():
  s = []
  while True:
    data = ser.readLine()
    decoded_bytes = int(data.decode("utf-8"))
    if data:
      s.append(decoded_bytes)
    if len(s) == 1:
      angle = s[0]
      print(angle)
    if len(s) == 1:
      s = []
