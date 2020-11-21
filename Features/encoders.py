import time
import serial

# Arduino Communication Encoder
SERIAL_PORT = '/dev/ttyACM1'
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT,BAUD_RATE)


