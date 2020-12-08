from __future__ import division
from __future__ import print_function
from motors import *
from argparse import ArgumentParser
import numpy as np
import tagUtils as tud
import cv2
import threading
import time
import apriltag
import math
import pretty_errors
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
            setspeedm1(800)
            setspeedm2(800)
            right()
        elif x < 120:
            setspeedm1(800)
            setspeedm2(800)
            left()
        else:
            if x > 580:
                setspeedm1(800)
                setspeedm2(800)
                right()
            elif x < 540:
                setspeedm1(800)
                setspeedm2(800)
                left()
            else:
                stop()
                time.sleep(2)
                if x > 580:
                    setspeedm1(800)
                    setspeedm2(800)
                    right()
                elif x < 540:
                    setspeedm1(800)
                    setspeedm2(800)
                    left()
                    stop()
                else:
                    break

dis = 21
def robodock():
    global dis
    while True:
        robodock.k = 0
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections, dimg = detector.detect(gray, return_image=True)
        num_detections = len(detections)
        print('Detected {} tags.\n'.format(num_detections))
        setspeedm1(0)
        setspeedm2(0)
        for i, detection in enumerate(detections):
            #print('Detection {} of {}:'.format(i+1, num_detections))
            #print()
            #print(detection.tostring(indent=2))
            #print()
 
            dis = tud.get_distance(detection.homography,10000)
            print('Distance',dis)
            if (dis < 20):  
                break
        if (dis < 20):  
            break 
            detector.detection_pose(detection,camera_params,tag_size=1,z_sign=1)
          
            pose, e0, e1 = detector.detection_pose(detection,camera_params,tag_size)
            print(pose[2][0])
            if pose[2][0] < -0.1:
                #left()
                print(left)
            elif pose[2][0] > 0.1:
                #right()    
                if (dis >= 20):
                    setspeedm1(0)
                    setspeedm2(0)
                    #forward()
                elif (dis < 20):
                    stop()
                    print("stop")
                    k = k + 1
                    time.sleep(2)
            
        print(robodock.k)




#Start Camera in Thread
cupfinder = threading.Thread(target=camera_on)
cupfinder.start() 
while True:
    print("Running...")
    val = input()
    if val == 'moments':
        cup_finder()
        print(moments)
    elif val == 'area':
        cup_finder()
        print("Area of Cup: "+str(area))
    elif val == 'xy':
        cup_finder()
        print("Coordinates of Cup: ("+str(x)+","+str(y)+")")
    elif val == 'center':
        center()
    elif val == 'april':
        robodock()    
