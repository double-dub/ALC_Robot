from motors import *
import cv2
import numpy as np
import pretty_errors

cam = cv2.VideoCapture(0)

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

area = 0.0

def cup_finder():
    global area
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
        #print(area)
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
                setspeedm1(1200)
                setspeedm2(1200)
                right()
            elif (center_x - x) > max_x:
                cv2.line(frame,(int(x),int(y)),(center_x,center_y),(0,0,255),1)
                #print("left")
                setspeedm1(1200)
                setspeedm2(1200)
                left()
            else:
                stop()
                break
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
        #else:
            #cv2.rectangle(frame,start_point,end_point,(255,255,255),-1)
            #cv2.putText(frame,"Locating Cup...",font_pos,cv2.FONT_HERSHEY_SIMPLEX,font_scale,(0,0,0))
            print("Locating Cup...")
            #left()
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

cup_finder()
print(area)
