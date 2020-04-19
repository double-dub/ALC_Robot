# Robot Game Plan

# 1. Start Robot by executing the Object detection function.
#    Object_detection loop breaks when object is found and within range,
#    causing robot to stop and the next function to run.
    
# 2. Execute Walter's manipulator function to pick object up. 
#    After manipulator function picks the object up, it then it breaks
#    the loop, moving onto the next function.
 
# 3. Execute Albert's function to navigate towards the homebase. 
#    Function breaks when apriltag is detected (means robot is within homebase range)
#    and moves into the docking stage.

# 4. Execute Apriltag docking function. Use Walter's same function backwards 
#    to drop object at homebase. Use apriltag detection to
#    properly orientate robot onto the homebase which then causes the loop in the
#    function to break and lets the robot charge for set time.

# 5. Execute Joseph's battery reader function. Reads battery and restarts process
#    when robot is charged up.

from __future__ import division
from __future__ import print_function
import cv2
import numpy as np
import math
import apriltag
import tagUtils as tud

vid = cv2.VideoCapture(0)

Ymin = 0
Ymax = 100
Umin = 30
Umax = 140
Vmin = 0
Vmax = 100

rangeMin= np.array([Ymin, Umin, Vmin], np.uint8)
rangeMax = np.array([Ymax, Umax, Vmax], np.uint8)

minArea = 50
width = 1280
height = 880
center_y = int(height/2)
center_x = int(width/2)
max_y = height/5
max_x = width/4

start_point = (0,1000)
end_point = (1280,900)
font_scale = 1.85
font_pos = (0,945)

def object_detection():
    best_outline=[]
    while True:
        ret, frame = vid.read()
        imgMedian = cv2.medianBlur(frame,1)
        imgYUV = cv2.cvtColor(imgMedian,cv2.COLOR_BGR2YUV)
        imgThresh = cv2.inRange(imgYUV, rangeMin, rangeMax)
        imgErode = cv2.erode(imgThresh, None, iterations = 3)
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
        
        if area >= minArea:
            x = int(moments['m10'] / moments['m00'])
            y = int(moments['m01'] / moments['m00'])
            cv2.circle(frame,(x,y),5,(0,255,0),-1)
            cv2.line(frame,(int(x),int(y)),(int(center_x),int(center_y)),(0,255,0),1)
            cv2.rectangle(frame,start_point,end_point,(255,255,255),-1)
            cv2.putText(frame,"Position: "+str(int(x))+" , "+str(int(y)),font_pos,cv2.FONT_HERSHEY_SIMPLEX,font_scale,(0,0,0))
            if (x - center_x) > max_x:
                cv2.line(frame,(int(x),int(y)),(center_x,center_y),(0,0,255),1)
                print("right")
            elif (center_x - x) > max_x:
                cv2.line(frame,(int(x),int(y)),(center_x,center_y),(0,0,255),1)
                print("left")
            else:
                if (area <= 26000):
                    cv2.circle(frame,(int(x),int(y)),5,(255,0,0),-1)
                    print("forward")
                elif (area >= 28000):
                    cv2.circle(frame,(int(x),int(y)),5,(0,0,255),-1)
                    print("backward")
                else:
                    print("stop")
                    break
        else:
            cv2.rectangle(frame,start_point,end_point,(255,255,255),-1)
            cv2.putText(frame,"Searching For Object...",font_pos,cv2.FONT_HERSHEY_SIMPLEX,font_scale,(0,0,0))
            print("Searching For Object...")
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        cv2.namedWindow("Erode",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Erode",800,600)
        cv2.imshow("Erode",processed_img)
        cv2.namedWindow("Video Output",cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Video Output",800,600)
        cv2.imshow("Video Output",frame)
    vid.release()
    cv2.destroyAllWindows()


def apriltag_docking():
    vid = cv2.VideoCapture(0)

    window = 'Camera'
    cv2.namedWindow(window,cv2.WINDOW_NORMAL)

    detector = apriltag.Detector()

    while True:

        success, frame = vid.read()
        if not success:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        detections, dimg = detector.detect(gray, return_image=True)

        num_detections = len(detections)
        print('Detected {} tags.\n'.format(num_detections))
        
        for i, detection in enumerate(detections):
            print('Detection {} of {}:'.format(i+1, num_detections))
            #print()
            #print(detection.tostring(indent=2))
            #print()
            #print(detection.homography)
            print()

            # Homography broken down into rows
            row1 = [round(floater,4) for floater in detection.homography[0]]
            row2 = [round(floater,4) for floater in detection.homography[1]]
            row3 = [round(floater,4) for floater in detection.homography[2]]
            print("Homography Rows")
            print("Row 1: ",row1)
            print("Row 2: ",row2)
            print("Row 3: ",row3)
            print()

            # Homography broken down into columns
            print("Homography Columns")
            print("Column 1: ",
                  "\n",  
                  round(detection.homography[0][0],4),
                  "\n",
                  round(detection.homography[1][0],4),
                  "\n",
                  round(detection.homography[2][0],4))
            print()
            print("Column 2: ",
                  "\n",
                  round(detection.homography[0][1],4),
                  "\n",
                  round(detection.homography[1][1],4),
                  "\n",
                  round(detection.homography[2][1],4))
            print()
            print("Column 3: ",
                  "\n",
                  round(detection.homography[0][2],4),
                  "\n",
                  round(detection.homography[1][2],4),
                  "\n",
                  round(detection.homography[2][2],4))

            # Still working on orientation using homography matrix
            
            print()
            dis = tud.get_distance(detection.homography,10000)
            print('distance: ',dis)
            if dis > 350:
                print("Drive Forward")
            elif dis < 250:
                print("Drive Backward")
            else:
                print("Stop")

        overlay = frame // 2 + dimg[:, :, None] // 2

        cv2.resizeWindow(window,800,600)
        cv2.imshow(window, overlay)
        k = cv2.waitKey(1)

        if k == 27:
            break


# Start Robot by Searching for Object
object_detection()

# Walter's Manipulator function will go here

# Albert's Navigation towards homebase will go here

# Docking function when near homebase
apriltag_docking()

# Joseph's battery reader function will go here.
