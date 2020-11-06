#!/usr/bin/env python

'''Demonstrate Python wrapper of C apriltag library by running on camera frames.'''
from __future__ import division
from __future__ import print_function

from argparse import ArgumentParser
import cv2
import apriltag
import tagUtils as tud

cap = cv2.VideoCapture(0)
detector = apriltag.Detector()
window = 'Camera'

while True:

    success, frame = cap.read()
    if not success:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    detections, dimg = detector.detect(gray, return_image=True)

    num_detections = len(detections)
    print('Detected {} tags.\n'.format(num_detections))

    for i, detection in enumerate(detections):
        print('Detection {} of {}:'.format(i+1, num_detections))
        print()
        print(detection.tostring(indent=2))
        print()

        dis = tud.get_distance(detection.homography,10000)
        print('Distance',dis)
    
    overlay = frame // 2 + dimg[:, :, None] // 2
    cv2.namedWindow(window,cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window,800,600)
    cv2.imshow(window, overlay)
    k = cv2.waitKey(1)

    if k == 27:
        break


