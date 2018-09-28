# -*- coding: utf-8 -*-
"""
Created on Sat Aug 25 10:28:51 2018

@author: SQ
"""
import cv2 
import numpy as np
import imutils
import time 


camera=cv2.VideoCapture(1)
x = 1 # displays the frame rate every 1 second
counter = 0
                 
boundaries = [ ( [0, 43, 46],    #lower color range
                 [8, 255, 255] ) ]#upper color range 

start_time = time.time()

while True:
    ret,frame = camera.read()   #ret is boolean
   # resize the frame, blur it, and convert it to the HSV color space
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv=cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV) 

    for (lower, upper) in boundaries:
        lower = np.array(lower, dtype = "uint8") 
        upper = np.array(upper, dtype = "uint8") 
        
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)    

   # find contours in the mask and initialize the current
	# (x, y) center of the ball
    cnts= cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) 
    if imutils.is_cv2():
        cnts = cnts[0]
    else:
        cnts = cnts[1]
    center = None
    
    # only proceed if at least one contour was found
    if len(cnts)>0:
      # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
        c=max(cnts,key = cv2.contourArea)
        ((x,y),radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
      
    # only proceed if the radius meets a minimum size    
    if radius > 10 and radius < 200:
# draw the circle and centroid on the frame, then update the list of tracked points
        cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        x,y,radius = int(x),int(y),int(radius)
        
#        if y >= 150 and y <= 250:
            
#           print("center")
#        elif y > 250 and y < 500:
 #           print("up")
#        elif y > 0 and y < 150:
  #          print("down")
 #       print(x,y,radius)
 
 #   if (time.time() - start_time)>x:
    cv2.putText(frame,"FPS {0}".format(int(1/(time.time() - start_time))),(550,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,0,255),2)
    cv2.imshow("frame", frame) 
    start_time = time.time()

    if cv2.waitKey(1) ==27 :   #press "Esc" to stop
        break 
 
camera.release() 
cv2.destroyAllWindows() 