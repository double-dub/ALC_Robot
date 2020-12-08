'''Demonstrate Python wrapper of C apriltag library by running on camera frames.'''
from __future__ import division
from __future__ import print_function
from motors import *
from argparse import ArgumentParser
import numpy as np
import tagUtils as tud
import apriltag
import math
import cv2

cam = cv2.VideoCapture(0)
detector = apriltag.Detector()
window = 'Camera'
fx ,fy ,cx, cy = (1390.3977534277124, 1409.7720276829375, 639.50000001162925, 479.49999999786633)
camera_params = fx, fy, cx, cy
tag_size = 1

Ymin = 0
Ymax = 150
Umin = 0
Umax = 166
Vmin = 147
Vmax = 212

rangeMin= np.array([Ymin, Umin, Vmin], np.uint8)
rangeMax = np.array([Ymax, Umax, Vmax], np.uint8)

minArea = 50
width = 1280
height = 880
center_y = int(height/2)
center_x = int(width/2)
max_y = height/6
max_x = width/5

start_point = (0,1000)
end_point = (1280,900)
font_scale = 1.85
font_pos = (0,945)

def robodock():
   while True:
       robodock.k = 0
       ret, frame = cam.read()
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
        
           detector.detection_pose(detection,camera_params,tag_size=1,z_sign=1)
        
           pose, e0, e1 = detector.detection_pose(detection,camera_params,tag_size)
           print(pose[2][0])
           if pose[2][0] < -0.1:
               setspeedm1(950)
               setspeedm2(950)
               left()
           elif pose[2][0] > 0.1:
               setspeedm1(950)
               setspeedm2(950)
               right()    
               if (dis >= 20):
                   setspeedm1(850)
                   setspeedm2(850)
                   forward()
               elif (dis < 20):
                   stop()
                   print("stop")
                   k = k + 1
                   time.sleep(2)
                   break
       print(robodock.k)
       #if k > 0:
       #    break
       #overlay = frame // 2 + dimg[:, :, None] // 2
       #cv2.namedWindow(window,cv2.WINDOW_NORMAL)
       #cv2.resizeWindow(window,800,600)
       #cv2.imshow(window, overlay)
       #k = cv2.waitKey(1)

       #if k == 27:
       #    break

def cup_finder():
    best_outline=[]
    while True:
        ret, frame = cam.read()
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
        print(area)
        if area >= minArea:
            x = int(moments['m10'] / moments['m00'])
            y = int(moments['m01'] / moments['m00'])
            #cv2.circle(frame,(x,y),5,(0,255,0),-1)
            #cv2.line(frame,(int(x),int(y)),(int(center_x),int(center_y)),(0,255,0),1)
            #cv2.rectangle(frame,start_point,end_point,(255,255,255),-1)
            #print("Position: x - "+str(int(x))+" y - "+str(int(y)))
            #cv2.putText(frame,"Position: "+str(int(x))+" , "+str(int(y)),font_pos,cv2.FONT_HERSHEY_SIMPLEX,font_scale,(0,0,0))
            if (x - center_x) > max_x:
                cv2.line(frame,(int(x),int(y)),(center_x,center_y),(0,0,255),1)
                #print("right")
                setspeedm1(1100)
                setspeedm2(1100)
                right()
            elif (center_x - x) > max_x:
                cv2.line(frame,(int(x),int(y)),(center_x,center_y),(0,0,255),1)
                #print("left")
                setspeedm1(1100)
                setspeedm2(1100)
                left()
            else:
                fwrd_short()
                create_vector()
                enc_res()
                print(vector_list)
                if (area <= 158400):
                    cv2.circle(frame,(int(x),int(y)),5,(255,0,0),-1)
                    #print("forward")
                    setspeedm1(850)
                    setspeedm2(850)
                    #forward()
                elif (area >= 234400):
                    cv2.circle(frame,(int(x),int(y)),5,(0,0,255),-1)
                    #print("backward")
                    setspeedm1(850)
                    setspeedm2(850)
                    #backward()
                else:
                    #print("stop")
                    setspeedm1(0)
                    setspeedm2(0)
                    #stop()
                    print("Cup Localized!")
                    break
        else:
            #cv2.rectangle(frame,start_point,end_point,(255,255,255),-1)
            #cv2.putText(frame,"Locating Cup...",font_pos,cv2.FONT_HERSHEY_SIMPLEX,font_scale,(0,0,0))
            print("Locating Cup...")
            setspeedm1(1000)
            setspeedm2(1000)
            left()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        #cv2.namedWindow("Erode",cv2.WINDOW_NORMAL)
        #cv2.resizeWindow("Erode",800,600)
        #cv2.imshow("Erode",processed_img)
        #cv2.namedWindow("Video Output",cv2.WINDOW_NORMAL)
        #cv2.resizeWindow("Video Output",800,600)
        #cv2.imshow("Video Output",frame)
    cam.release()
    cv2.destroyAllWindows()
