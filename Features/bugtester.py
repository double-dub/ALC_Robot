from __future__ import division
from __future__ import print_function
from motors import *
from battery import *
from imu import *
from manipulator import *
from resultant_vector import *
from argparse import ArgumentParser
from ultra import *
import threading
import pretty_errors
import numpy as np
import tagUtils as tud
import apriltag
import math
import cv2


angle = 0.0
vector_list = []
encoder1 = 0
encoder2 = 0
#initialize video capture
cam = cv2.VideoCapture(0)
width = 1280
height = 880
#Apriltag Detector Initialize
detector = apriltag.Detector()
window = 'Camera'
fx ,fy ,cx, cy = (1390.3977534277124, 1409.7720276829375, 639.50000001162925, 479.49999999786633)
camera_params = fx, fy, cx, cy
tag_size = 1
#Create color threshold & save into arrays
Ymin = 0
Ymax = 150
Umin = 0
Umax = 166
Vmin = 147
Vmax = 212
minArea = 50
rangeMin = np.array([Ymin, Umin, Vmin], np.uint8)
rangeMax = np.array([Ymax, Umax, Vmax], np.uint8)
#Global Variables
moments = 0
area = 0.0
x = 0
y = 0
ret,frame = cam.read()
area = 0
dis = 21

class PID:
    """PID Controller
    """

    def __init__(self, P=5.5 , I=1, D=1, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time


def rightpid():
    global angle 
    rangle = angle
    goala = rangle + 90
    pid = PID()
    pid.SetPoint = goala
    pid.setSampleTime(1)
    while True:
        pid.update(angle)
        targetPWM = pid.output
        #targetPWM = max(min(int(targetPWM), 1300),800)
        print("Goal Angle: "+str(goala))
        print("Current Angle: "+str(angle))
        print("Target PWM: "+str(targetPWM))
        print("\n")
        speed = round(targetPWM + 550)
        print(speed)
        setspeedm1(speed)
        setspeedm2(speed)
        right()
        if (goala - angle) < 1.5:
            stop()
            break        


def leftpid():
    global angle
    rangle = angle
    goala = rangle - 90
    pid = PID()
    pid.SetPoint = goala
    pid.setSampleTime(1)
    while True:
        pid.update(angle)
        targetPWM = pid.output
        #targetPWM = max(min(int(targetPWM), 1300),800)
        print("Goal Angle: "+str(goala))
        print("Current Angle: "+str(angle))
        print("Target PWM: "+str(targetPWM))
        print("\n")
        speed = round(abs(targetPWM) + 550)
        print(speed)
        setspeedm1(speed)
        setspeedm2(speed)
        left()
        if abs(angle - goala) < 1.5:
            stop()
            break
 

def zoom(ticks):
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<ticks):
        setspeedm1(1500)
        setspeedm2(1500)
        forward()
    stop()
    print(encoder1)


def return_base():
    global angle
    global vector_list

    rangle = angle
    
    res_mag, res_deg = calc_resultant(vector_list)
    if (res_deg < 0):
        res_deg = res_deg + 360
    res_deg = res_deg + 180 % 360
    pid = PID()
    pid.SetPoint = res_deg
    pid.setSampleTime(1)
    while True:
        pid.update(angle)
        targetPWM = pid.output
        #targetPWM = max(min(int(targetPWM), 1300),800)
        print("Goal Angle: "+str(res_deg))
        print("Current Angle: "+str(angle))
        print("Target PWM: "+str(targetPWM))
        print("\n")
        speed = round(abs(targetPWM) + 500)
        print(speed)
        setspeedm1(750)
        setspeedm2(750)
        left()
        if abs(res_deg - angle) < 1.5:
            stop()
            break
    print(res_mag)
    zoom(res_mag-4000) 

def create_vector():
    time.sleep(2)
    global encoder1, encoder2
    global angle
    global vector_list

    enc_avg = round((encoder1 + encoder2) / 2)
    vector_list.append([enc_avg, angle])
    time.sleep(1)


def snaked():
    
    fwrdd()
    create_vector()
    enc_res()
    rightpid()
    time.sleep(1)
    enc_res()
    fwrd_short()
    create_vector()
    enc_res()    
    rightpid()
    enc_res()
    time.sleep(1)
    
    fwrdd()
    create_vector()
    enc_res()    
    leftpid()
    enc_res()

    time.sleep(1)
    fwrd_short()
    create_vector()    
    enc_res()    
    leftpid()
    enc_res()

    time.sleep(1)    
    fwrdd()
    create_vector()
    enc_res()    
    time.sleep(2)
    
    #Return to Base
    print(vector_list)
    return_base()


def snake():
    global area    
    
    fwrd()
    create_vector()
    #Check if Object in Sight
    if area != 0:
        center()
        return
    create_vector()
    enc_res()
    rightpid()
    time.sleep(2)
    enc_res()
    fwrd_short()
    create_vector()
    enc_res()    
    rightpid()
    enc_res()
    time.sleep(2)
    
    fwrd()
    create_vector()
    #Check if Object in Sight
    if area != 0:
        center()
        return
    enc_res()    
    leftpid()
    enc_res()
    time.sleep(2)
    fwrd_short()
    create_vector()    
    enc_res()    
    leftpid()
    enc_res()
    time.sleep(2)
    
    fwrd()
    create_vector()
    #Check if Object in Sight
    if area != 0:
        center()
        return
    enc_res()    
    time.sleep(2)
    
    #Return to Base
    print(vector_list)
    return_base()


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


def enc_res():
    serial_e.write(b"0")
    print("Reset")


def fwrd():
    global encoder1
    global area
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<10000):
        setspeedm1(1200)
        setspeedm2(1200)
        forward()
        print(encoder1)
        cup_finder()
        if area != 0:
            break
    stop()
    print(encoder1)

def fwrdd():
    global encoder1
    global area
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<11500):
        setspeedm1(1200)
        setspeedm2(1200)
        forward()
        print(encoder1)
    stop()
    print(encoder1)


def fwrd_cup(ticks):
    global encoder1
    global area
    while (encoder1 < ticks):
        setspeedm1(1200)
        setspeedm2(1200)
        forward()
        print(encoder1)
        cup_finder()
        if area != 0:
            break
    stop()
    print(encoder1)


def fwrd_short():
    global encoder1
    #10186 ticks = 5ft with 2.47in radius
    while (encoder1<6000):
        setspeedm1(1200)
        setspeedm2(1200)
        forward()
    stop()
    print(encoder1)


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


def camera_on():
    global ret,frame
    while True:
        ret, frame = cam.read()
'''
                if (area <= 158400):
                    cv2.circle(frame,(int(x),int(y)),5,(255,0,0),-1)
                    #print("forward")
                    setspeedm1(1100)
                    setspeedm2(1100) 
                    #forward()
                elif (area >= 234400):
                    cv2.circle(frame,(int(x),int(y)),5,(0,0,255),-1)
                    #print("backward")
                    setspeedm1(1100)
                    setspeedm2(1100) 
                    #backward()
                else:
                    #print("stop")
                    setspeedm1(0)
                    setspeedm2(0)
                    stop()
                    print("Cup Localized!")
                    break
'''
def cup_finder():
    global moments
    global area
    global x
    global y
    best_outline=[]
    counter = 0
    while True:
        imgMedian = cv2.medianBlur(frame,5)
        imgYUV = cv2.cvtColor(imgMedian,cv2.COLOR_BGR2YUV)
        imgThresh = cv2.inRange(imgYUV, rangeMin, rangeMax)
        imgErode = cv2.erode(imgThresh, None, iterations = 3)
        imgDilate = cv2.dilate(imgErode, None, iterations = 10)
        processed_img = imgErode
        contours,hierarchy = cv2.findContours(processed_img,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        major_area = 0
        for contour_position in contours:
            area = cv2.contourArea(contour_position)
            if (area > major_area):
                major_area = area
                best_outline = contour_position

        moments = cv2.moments(np.array(best_outline))

        area = moments['m00']
        #print(area)
        if area >= minArea:
            x = int(moments['m10'] / moments['m00'])
            y = int(moments['m01'] / moments['m00'])
        counter = counter + 1
        if counter > 3:
            break


#Call center every once in a while from different distances
#be lenient when distance is far
def center():
    while True:
        cup_finder()
        if x > 900:
            setspeedm1(750)
            setspeedm2(750)
            right()
        elif x < 120:
            setspeedm1(750)
            setspeedm2(750)
            left()
        else:
            stop()
            time.sleep(.5)
            if x > 580:
                setspeedm1(700)
                setspeedm2(700)
                right()
            elif x < 540:
                setspeedm1(700)
                setspeedm2(700)
                left()
            else:
                stop()
                time.sleep(.5)
                if x > 580:
                    setspeedm1(700)
                    setspeedm2(700)
                    right()
                elif x < 540:
                    setspeedm1(700)
                    setspeedm2(700)
                    left()
                    stop()
                else:
                    break


def robodock():
    global dis
    while True:
        robodock.k = 0
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections, dimg = detector.detect(gray, return_image=True)
        num_detections = len(detections)
        print('Detected {} tags.\n'.format(num_detections))
        setspeedm1(800)
        setspeedm2(800)
        right()
        for i, detection in enumerate(detections):
            #print('Detection {} of {}:'.format(i+1, num_detections))
            #print()
            #print(detection.tostring(indent=2))
            #print()
 
            dis = tud.get_distance(detection.homography,10000)
            print('Distance',dis)
            #if (dis < 20):  
            #    break
       # if (dis < 20):  
        #    break 
            detector.detection_pose(detection,camera_params,tag_size=1,z_sign=1)
          
            pose, e0, e1 = detector.detection_pose(detection,camera_params,tag_size)
            print(pose[2][0])
            if pose[2][0] < -0.2:
                setspeedm1(800)
                setspeedm2(800)
                right()
            elif pose[2][0] > 0.2:
                setspeedm1(800)
                setspeedm2(800)
                left()
            else:
                stop()
                break    
               # if (dis >= 20):
               #     setspeedm1(0)
               #     setspeedm2(0)
                    #forward()
               # elif (dis < 20):
               #     stop()
               #     print("stop")
               #     k = k + 1
               #     time.sleep(2)
        print(robodock.k)


def d2cup():
    global area
    cup_finder()
    start_area = area
    area2cup = 383000 - start_area
    n = area2cup / 8
    goal_area = area + n
    dis = distance()
    for i in range(4):
        speed = 800
        setspeedm1(speed)
        setspeedm2(speed)
        print(i)
        if area > 383000:
            break
        while area < goal_area:
            cup_finder()
            print(area)
            forward()
        stop()
        time.sleep(2)
        center()
        #probably record vector here
        cup_finder()
        print("Area "+str(area))
        goal_area = area + n
        print("Goal Area "+str(goal_area))
    time.sleep(1)
    


def pickup_cup():
    rest()
    extend_cup()
    grab_cup()
    retrieve_cup()
 
def aread():
    global area
    stop = 0
    cm = 383000 - area
    cm = round(cm/1000)
    return cm
    if area < 383000:
        return stop 

def tickd():
    total = 0
    for i in range(5):
        dist = distance()
        print(dist)
        total = total + dist
    total = total / 5
    print(total)
    dis = round(total - 10)
    ticks = (dis * 91.12)
    print(ticks)


#Threaded Serial Readers
imu_th = threading.Thread(target=imu_thread)
enc_th = threading.Thread(target=encoder_thread)
cupfinder = threading.Thread(target=camera_on)
imu_th.start()
enc_th.start()
cupfinder.start()  

while True:
    print("\n")
    print("Debug Program/Tester...\n")
    print("Use the following keys to run a function:\n")
    print("Motor Functions: ")
    print("Forward - w  Backward - s  Right - d  Left - a  Stop - q")
    print("Check Battery: ")
    print("Battery% - b  Read IMU - l  Read Encoders - e  Reset Encoders - er\n")
    print("Manipulator Functions: ")
    print("Extend - f  Grab - g  Retrieve - r SpinDrop - t Rest - y\n")
    print("Machine Vision Functions: ")
    print("Cup Finder - z  Cup Area - area  Cup Distance - ultra")  
    print("Cup Coordinates - xy  Center Robot - center  Apriltag - x")
    print("\n\n")
    
    value = input()
    if value == 'w':
        setspeedm1(1000)
        setspeedm2(1000)
        print("Forward")
        forward()
        print("\n")
    elif value == 'snake':
        snaked()
        robodock()
        print("\n")
    elif value == 'demo':
        #dis = distance()
        ticks = tickd()
        cm = aread() 
        #do snake
        snake()
        #get to cup
        #save res vector as getting to cup
        d2cup()
        #use manip to pick cup
        pickup_cup()
        #resultant vector to hit the homebase
        #return_base()
        #use apriltag to dock properly
        #robodock()
        print("\n")
    elif value == 'snake':
        snake()
        print("\n")
    elif value == 's':
        setspeedm1(1000)
        setspeedm2(1000)
        backward()
        print("Backward")
        print("\n")
    elif value == 'd':
        setspeedm1(1300)
        setspeedm2(1300)
        print("Right")
        right()
        print("\n")
    elif value == 'a':
        setspeedm1(1300)
        setspeedm2(1300)
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
        print("\n")
    elif value == 'g':
        print("Grab")
        grab_cup()
        print("\n")
    elif value == 'r':
        print("Retrieve")
        retrieve_cup()
        print("\n")
    elif value == 't':
        print("Spin Drop")
        spindrop()
        print("\n")
    elif value == 'y':
        print("Rest")
        rest()
        print("\n")
    elif value == 'z':
        print("Give me the cup!")
        cup_finder()
        print("\n")
    elif value == 'x':
        print("Apriltag Docking")
        robodock()
        print("\n")
    elif value == 'l':
        print("Current Angle: "+str(angle))
        print("\n")
    elif value == 'e':
        print("Encoder1: "+str(encoder1))   
        print("Encoder2: "+str(encoder2))   
        print("\n")
    elif value == 'er':
        enc_res()
        print("\n")
    elif value == 'ultra':
        dist = distance()
        print("Measured Distance = %.1f cm" % dist) 
        print("\n")
    elif value == 'moments':
        cup_finder()
        print(moments)
        print("\n")
    elif value == 'd2cup':
        cup_finder()
        d2cup()
        print("\n")
    elif value == 'avgdis':
        tickd()
        print("\n")
    elif value == 'aread':
        cup_finder()
        print(aread())
        print("\n")
    elif value == 'area':
        cup_finder()
        print("Area of Cup: "+str(area))
        print("\n")
    elif value == 'xy':
        cup_finder()
        print("Coordinates of Cup: ("+str(x)+","+str(y)+")")
        print("\n")
    elif value == 'center':
        center()
        print("\n")
    elif value == 'april':
        robodock() 
        print("\n")
    else:
        print("<<< Invalid Key Entry >>>")
        print("\n")
    print("\n\n")
