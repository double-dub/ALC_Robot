from motors import *
from battery import *
from encoders import *

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

