from motors import *
from battery import *
from encoders import *
#from color_object import *
from robot_dock import *
from imu import *
from manipulator import *
import pretty_errors

while True:
    print("\n")
    print("Debug Program...Press spacebar to quit\n")
    print("Use the following keys to run a function:\n")
    print("Motor Functions: ")
    print("Forward - w  Backward - s  Right - d  Left - a  Stop - q")
    print("Check Battery: ")
    print("Battery% - b Read IMU - l")
    print("Manipulator Functions: ")
    print("Extend - f  Grab - g  Retrieve - r SpinDrop - t Rest - y")
    print("Cup Finder - z Apriltag - x\n")

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
        print("Manipulator Extend")
        extend_cup()
    elif value == 'g':
        print("Grab")
        grab_cup()
    elif value == 'r':
        print("Retrieve")
        retrieve_cup()
    elif value == 't':
        print("Spin Drop")
        spindrop()
    elif value == 'y':
        print("Rest")
        rest()
    elif value == 'z':
        print("Give me the cup!")
        cup_finder()
    elif value == 'x':
        print("Apriltag Docking")
        robodock()
    elif value == 'k':
        start_imu()
    elif value == 'l':
        print("IMU readings: ")
        start_imu()
        time.sleep(.01)
        halt_imu()
        imu_read()
    elif value =='n':
        get_angle()
    elif value == 'j':
        halt_imu()
    elif value == ' ':
        break
    else:
        print("<<< Invalid Key Entry >>>")
        print("\n")

