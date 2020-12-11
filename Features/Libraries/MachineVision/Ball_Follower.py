from motors import *
import time
import numpy as np
import math
import cv2
import pretty_errors

cap = cv2.VideoCapture(1)

#Horizontal adjustments to center object    
def adjustment(x,y,max_x,center_x):
    #If distance on right-side is > limit, adjust
    if (x - center_x) > max_x:
        cv2.line(c_input,(int(x),int(y)),(center_x,center_y),(0,0,255),1)
        print("Right")
        right()
        time.sleep(0.05)
        stop()
    #If distance on left-side is > limit, adjust
    elif (center_x - x) > max_x:
        cv2.line(c_input,(int(x),int(y)),(center_x,center_y),(0,0,255),1)
        print("Left")
        left()
        time.sleep(0.05)
        stop()
    else:
        stop()
     
#HSV values for object
Hmin = 42
Hmax = 92
Smin = 62
Smax = 255
Vmin = 63
Vmax = 235

#HSV's range matrices
rangeMin = np.array([Hmin, Smin, Vmin], np.uint8)
rangeMax = np.array([Hmax, Smax, Vmax], np.uint8)

#Image processing to isolate object
def process(c_input):
    imgMedian = cv2.medianBlur(c_input,1)
    imgHSV = cv2.cvtColor(imgMedian,cv2.COLOR_BGR2HSV)
    imgThresh = cv2.inRange(imgHSV, rangeMin, rangeMax)
    imgErode = cv2.erode(imgThresh, None, iterations = 3)
    return imgErode

cv2.namedWindow("Erode")

#Object Parameters
minArea = 50
width = 160
height = 160
center_y = int(height/2)
center_x = int(width/2)
max_y = height/5 
max_x = width/4
best_outline=[]

#Sets size of active windows
if cap.isOpened():
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
while True:
    #Image transforms to isolate object
    ret, c_input = cap.read()
    processed_img = process(c_input)
    cv2.imshow("Erode", processed_img)
    contours,hierarchy = cv2.findContours(processed_img,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
    imgHSV = cv2.cvtColor(c_input,cv2.COLOR_BGR2HSV)   
    imgThresh = cv2.inRange(imgHSV, rangeMin, rangeMax)
    imgErode = cv2.erode(imgThresh, None, iterations = 3)
    major_area = 0
    for contour_position in contours:
        area = cv2.contourArea(contour_position)
        if (area > major_area):
            major_area = area
            best_outline = contour_position
    
    moments = cv2.moments(np.array(best_outline))
    area = moments['m00']
    
    if area >= minArea:
        x = int(moments['m10'] / moments['m00'])
        y = int(moments['m01'] / moments['m00'])
    
    #Mark center of object
        cv2.circle(c_input,(x,y),5,(0,255,0),-1)
    
    #Potition of object in relation to the center
        cv2.line(c_input,(int(x),int(y)),(int(center_x),int(center_y)),(0,255,0),1)
    
    #Text rectangle/background
        cv2.rectangle(c_input,(0,160),(160,145),(255,255,255),-1)
    
    #Footer position text
        cv2.putText(c_input,"Position: "+str(int(x))+" , "+str(int(y)),(0,156),cv2.FONT_HERSHEY_COMPLEX_SMALL,.5,(0,0,0))
    #Executes to move robot towards object if parameters are met    
        adjustment(x,y,max_x,center_x)
        if (area<=1000):
            cv2.circle(c_input, (int(x), int(y)),5,(255,0,0),-1)
            print("Forward")
            forward()
        elif (area>=2000):
            cv2.circle(c_input, (int(x), int(y)),5,(0,0,255),-1)
            print("Backward")
            backward()
        else:
            print("Stop")
            stop()
    
    else:
        # Display text for when no object in sight
        cv2.rectangle(c_input,(0,160),(160,145),(255,255,255),-1)
        cv2.putText(c_input,"No Object In Sight",(0,156),cv2.FONT_HERSHEY_COMPLEX_SMALL,.5,(0,0,0))
        print("Searching...")
        left()
    
    #Displays normal camera input with overlays
    cv2.imshow("Input",c_input)
    cv2.waitKey(10)
