import time
import serial

# Arduino Communication Encoder
SERIAL_PORT_E = '/dev/ttyACM1'
BAUD_RATE = 9600
serial_e = serial.Serial(SERIAL_PORT_E,BAUD_RATE)


def enc_res():
    serial_e.write(b"0")
    print("Reset")


def encoder_thread():
    global encoder1
    global encoder2
    elist = []
    while True:
        while serial_e != 0:
            data = serial_e.readline()
            decoded_bytes = data[0:len(data)-2].decode("utf-8")
            if data:
                elist.append(int(decoded_bytes))
            if len(elist) == 2:
                encoder1 = elist[0]
                encoder2 = elist[1]
            if len(elist) == 2:
                elist = []


def fwrd():
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<10000):
        setspeedm1(1400)
        setspeedm2(1400)
        forward()
    stop()
    print(encoder1)


def fwrd_short():
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<6943):
        setspeedm1(1400)
        setspeedm2(1400)
        forward()
    stop()
    print(encoder1)

