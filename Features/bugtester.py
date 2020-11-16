from motors import *
from battery import *
from encoders import *

def encoder_pid():
    p = []
    kp = .2
    while True:
        data = ser.readline()
        decoded_bytes = int(data[0:len(data)-2].decode("utf-8"))
        if data:
            p.append(decoded_bytes)
        if len(p) == 2:
            speed = 1000
            slave_pwm = speed
            pwm.set_pwm(pwm1,0,1000)
            enc1 = p[0]
            enc2 = p[1]
            enc = [[],[]]
            enc[0].append(enc1)
            enc[1].append(enc2)
            print(enc)
            error = enc1 - enc2
            slave_pwm += round(error / kp)
            pwm.set_pwm(pwm2,0,slave_pwm)
        if len(p) == 2:
            p = []

def one_rot():
    p = []
    kp = .2
    while True:
        data = ser.readline()
        decoded_bytes = int(data[0:len(data)-2].decode("utf-8"))
        if data:
            p.append(decoded_bytes)
        if len(p) == 2:
            speed = 1000
            slave_pwm = speed
            pwm.set_pwm(pwm1,0,1000)
            enc1 = p[0]
            enc2 = p[1]
            enc = [[],[]]
            enc[0].append(enc1)
            enc[1].append(enc2)
            print(enc)
            error = enc1 - enc2
            slave_pwm += round(error / kp)
            pwm.set_pwm(pwm2,0,slave_pwm)
            if (enc1 >= 3592) or (enc2 >= 3592):
                stop()
                time.sleep(1)
                ser.write("Reset".encode("utf-8"))
                ser.flush()
                print("Encoder Reset")
        if len(p) == 2:
            p = []   


def encoder_read():
    s = []
    ser.write("Reset".encode("utf-8"))
    ser.readline()
    while True:
        data = ser.readline()
        decoded_bytes = int(data[0:len(data)-2].decode("utf-8"))
        if data:
            s.append(decoded_bytes)
        if len(s) == 2:
            enc1 = s[0]
            enc2 = s[1]
            enc = [[],[]]
            enc[0].append(enc1)
            enc[1].append(enc2)
            print(enc)
        if len(s) == 2:
            s = []



print("\n")
print("Debug Program...Press spacebar to quit\n")
print("Use the following keys to run a function:\n",end="\nMotor Functions: ")
print("Forward - w  Backward - s  Right - d  Left - a  Stop - q",end="\nCheck Battery: ")
print("Battery% - b",end="\nManipulator Functions: ")
print("Extend - f  Grab - g  Retrieve - r SpinDrop - t ")

while True:
    value = input()
    if value == 'w':
        setspeed(1000)
        print("Forward")
        forward()
        print("\n")
    elif value == 's':
        setspeed(1000)
        backward()
        print("Backward")
        print("\n")
    elif value == 'd':
        setspeed(1300)
        print("Right")
        right()
        print("\n")
    elif value == 'a':
        setspeed(1300)
        print("Left")
        left()
        print("\n")
    elif value == 'q':
        print("Stop")
        stop()
        print("\n")
    elif value == 'b':
        print("Voltage:%5.2fV" % readVoltage(bus))
        print("Battery:%5i%%" % readCapacity(bus))
    elif value == 'f':
        print("function")
    elif value == 'g':
        print("function")
    elif value == 'r':
        print("function")
    elif value == 't':
        print("function")
    
    
    elif value == ' ':
        break
    else:
        print("<<< Invalid Key Entry >>>")
        print("\n")

