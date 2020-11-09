from __future__ import division
import numpy as np
import RPi.GPIO as gpio
import Adafruit_PCA9685
import cv2
import math
import time

#GPIO setup
# Initialize PCA9685 Board
pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=0)

#initiate gpio pins & ignore warnings
gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)

#Inverse direction pin (low=forward, high=backward)
inv = 31
gpio.setup(inv,gpio.OUT)

# RIGHT WHEEL
#motor A initialize
#in1 and in2
m1_in1 = 29
m1_in2 = 23
gpio.setup(m1_in1,gpio.OUT)
gpio.setup(m1_in2,gpio.OUT)

# LEFT WHEEL
#motor B initialize
#in1 and in2
m2_in1 = 19
m2_in2 = 21
gpio.setup(m2_in1,gpio.OUT)
gpio.setup(m2_in2,gpio.OUT)

# PWM pins
pwm1 = 9
pwm2 = 8
pwm.set_pwm(pwm1,0,1000)
pwm.set_pwm(pwm2,0,1000)

#Values for Object
Ymin = 0
Ymax = 179
Umin = 30
Umax = 255
Vmin = 0
Vmax = 122

#YUV Range of Values Matrices
rangeMin= np.array([Ymin, Umin, Vmin], np.uint8)
rangeMax = np.array([Ymax, Umax, Vmax], np.uint8)

#Start Video Capture
vid = cv2.VideoCapture(0)

#Define Video Capture Width & Height
width = 1280
height = 880
#Define Important Parameters for Centering
minArea = 50
center_y = int(height/2)
center_x = int(width/2)
max_y = height/5
max_x = width/4
best_outline=[]

#Text-Box Sizing Parameters
start_point = (0,1000)
end_point = (1280,900)
font_scale = 1.85
font_pos = (0,945)

def backward():
    #motor A
    gpio.output(m1_in1,gpio.LOW)
    gpio.output(m1_in2,gpio.HIGH)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.HIGH)

def forward():
    #motor A
    gpio.output(m1_in1,gpio.LOW)
    gpio.output(m1_in2,gpio.HIGH)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.LOW)

def stop():
    #motor A
    gpio.output(m1_in1,gpio.LOW)
    gpio.output(m1_in2,gpio.LOW)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.LOW)

def left():
    #motor A
    gpio.output(m1_in1,gpio.HIGH)
    gpio.output(m1_in2,gpio.LOW)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.HIGH)

def right():
    #motor A
    gpio.output(m1_in1,gpio.HIGH)
    gpio.output(m1_in2,gpio.LOW)
    #motor B
    gpio.output(m2_in1,gpio.LOW)
    gpio.output(m2_in2,gpio.HIGH)
    gpio.output(inv,gpio.LOW)


#Robot Movement/Camera_Adjustment Function
def adjustment(x,y,max_x,center_x):
    if (x - center_x) > max_x:
        cv2.line(frame,(int(x),int(y)),(center_x,center_y),(0,0,255),1)
        #right()
    elif (center_x - x) > max_x:
        cv2.line(frame,(int(x),int(y)),(center_x,center_y),(0,0,255),1)
        #left()
    else:
        if (area <= 1000):
            cv2.circle(frame,(int(x),int(y)),5,(255,0,0),-1)
            #forward()
        elif (area >=1500):
            cv2.circle(frame,(int(x),int(y)),5,(0,0,255),-1)
            #backward()
        else:
            #stop()
            print("Stop")

def process(frame):
    kernel = np.ones((5,5),np.uint8)
    imgMedian = cv2.medianBlur(frame,1)
    imgYUV = cv2.cvtColor(imgMedian,cv2.COLOR_BGR2YUV)
    imgThresh = cv2.inRange(imgYUV, rangeMin, rangeMax)
    imgErode = cv2.erode(imgThresh, None, iterations = 9)
    imgClose = cv2.morphologyEx(imgErode, cv2.MORPH_CLOSE,kernel)
    return imgClose

while True:
    #Creates a Camera Output to Add Overlays and Info. Onto
    ret, frame = vid.read()
    processed_img = process(frame)

    #Outline/Contour Creation of Detected Object
    contours,hierarchy = cv2.findContours(processed_img,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    major_area = 0
    for contour_position in contours:
        area = cv2.contourArea(contour_position)
        if (area > major_area):
            major_area = area
            best_outline = contour_position

    #Function That Finds Center of Object Contour
    moments = cv2.moments(np.array(best_outline))
    area = moments['m00']
    if area >= minArea:
        x = int(moments['m10'] / moments['m00'])
        y = int(moments['m01'] / moments['m00'])
        #Draw Circle @ Center of Object
        cv2.circle(frame,(x,y),5,(0,255,0),-1)
        #Draw Line From Center of Video Output to Object
        cv2.line(frame,(int(x),int(y)),(int(center_x),int(center_y)),(0,255,0),1)
        #Text Box W/ Info
        cv2.rectangle(frame,start_point,end_point,(255,255,255),-1)
        cv2.putText(frame,"Position: "+str(int(x))+" , "+str(int(y)),font_pos,cv2.FONT_HERSHEY_SIMPLEX,font_scale,(0,0,0))

        #Call Robot Movement/Adjustment Function
        adjustment(x,y,max_x,center_x)

    else:
        #Search for Object if None in Sight
        cv2.rectangle(frame,start_point,end_point,(255,255,255),-1)
        cv2.putText(frame,"Searching For Object...",font_pos,cv2.FONT_HERSHEY_SIMPLEX,font_scale,(0,0,0))
        print("Searching For Object...")
        #left()
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    #Display Detected Contour Window and Video Output Window
    cv2.namedWindow("Erode",cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Erode",800,600)
    cv2.imshow("Erode",processed_img)
    cv2.namedWindow("Video Output",cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Video Output",800,600)
    cv2.imshow("Video Output",frame)

vid.release()
cv2.destroyAllWindows()
