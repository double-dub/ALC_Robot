import time
import serial

# Arduino Communication Encoder
SERIAL_PORT_E = '/dev/ttyACM1'
BAUD_RATE = 9600
serial_e = serial.Serial(SERIAL_PORT_E,BAUD_RATE)
