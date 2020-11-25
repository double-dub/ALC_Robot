from encoders import *

def encoderthread():
    elist = []
    while True:
        while serial_e != 0:
            data = ser.readline()
            decoded_bytes = data[0:len(data)-2].decode("utf-8")
            if data:
                elist.append(int(decoded_bytes))
            if len(elist) == 2:
                encoderthread.enc1 = elist[0]
                encoderthread.enc2 = elist[1]
            if len(elist) == 2:
                elist = []
