# -*- coding: utf-8 -*-

import pigpio
import cv2 
import numpy as np
import imutils
import time
import serial
import RPi.GPIO as GPIO
import signal
import atexit
import gpiozero as gz


servopin1=23
servopin2=24
pi = pigpio.pi()
pi.set_mode(servopin1,pigpio.OUTPUT)
pi.set_mode(servopin2,pigpio.OUTPUT)
pi.set_PWM_frequency(servopin1,50)
pi.set_PWM_frequency(servopin2,50)
motor_r = gz.Motor(17,18)
motor_l = gz.Motor(27,22)

counter = 0
kp=float(0.5/150)
ball_x=0
plus1 = 1500
plus2 = 1200
search=1
search1=1
flag=0
target1=500
target2=500

camera=cv2.VideoCapture(0)
boundaries = [ ( [170, 43, 46],    #lower color range
                 [180, 255, 255] ) ]#upper color range 

start_time = time.time()

def change_speed(l,r):  
    motor_r.forward(r)
    motor_l.forward(l)
def turn_right(r,l):
    motor_r.forward(r)
    motor_l.backward(l)
def turn_left(r,l):
    motor_r.backward(r)
    motor_l.forward(l)
def back(r,l):
    motor_r.backward(r)
    motor_l.backward(l)
def stop():
        motor_r.stop()
        motor_l.stop()
stop()
while True:
    ret,frame = camera.read()   #ret is boolean
    frame = cv2.resize(frame, (300,300))
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv=cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV) 

    for (lower, upper) in boundaries:
        lower = np.array(lower, dtype = "uint8") 
        upper = np.array(upper, dtype = "uint8") 
        
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)    

    cnts= cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) 

    if imutils.is_cv2():
        cnts = cnts[0]
    else:
        cnts = cnts[1]
    center = None
        
    if len(cnts)>0:
        c=max(cnts,key = cv2.contourArea)
        ((x,y),radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        x,y,radius = int(x),int(y),int(radius)
        #print(radius)
        if radius > 1 and radius < 300:
            ball_x=x
            search=0
        else:
            ball_x=0


        if x >= 120 and x <= 180 and y>=120 and y<=180:
            flag=flag+1
            if flag==10:              
                error=plus1-1500
                error1=abs(error)
                search1=2
        elif  x > 180 and x < 300:
            if plus1>=510 and plus1 <=2500:
                plus1 = plus1-10
                pi.set_servo_pulsewidth(servopin1,plus1)
        elif  y > 180 and y < 300:
            if plus2>=1010 and plus2 <=1800:
                plus2 = plus2-10
                pi.set_servo_pulsewidth(servopin2,plus2)
                
        elif x >= 0 and x < 120 :
            if plus1>=500 and plus1 <= 2490:
                plus1 = plus1+10        
                pi.set_servo_pulsewidth(servopin1,plus1)      
        elif y >= 0 and y < 120 :
            if plus2>=1000 and plus2 <= 1790:
                plus2 = plus2+10        
                pi.set_servo_pulsewidth(servopin2,plus2)
    else:
        search=1
        
    if  search1 == 2:
        if error>0:
            if error1>0:
                print(error1)
                turn_right(0.9,0.9)
                error1=error1-48
            else:
                search=3
                stop()
        elif error<0:
            if error1>0:
                print(error1)                
                turn_left(0.9,0.9)
                error1=error1-48 
            else:
                stop()
                search=3
        else:
            stop()
            search=3
    if search == 3:    
        if plus1>1520:
            pi.set_servo_pulsewidth(servopin1,plus1)
            plus1=plus1-20
        elif plus1<1480:
            pi.set_servo_pulsewidth(servopin1,plus1)
            plus1=plus1+20                
        else:
            plus1=1500
            pi.set_servo_pulsewidth(servopin1,plus1)
            search=4
        
    if search == 1:       
        if plus1>target1:
            plus1=plus1-10
            pi.set_servo_pulsewidth(servopin1,plus1)
#            time.sleep(0.5)
        elif plus1<target1:
            plus1=plus1+10
            pi.set_servo_pulsewidth(servopin1,plus1)
#            time.sleep(0.5)       
        if plus2>target2:
            plus2=plus2-10
            pi.set_servo_pulsewidth(servopin2,plus2)
#            time.sleep(0.5)
        elif plus2<target2:
            plus2=plus2+10
            pi.set_servo_pulsewidth(servopin2,plus2)
#            time.sleep(0.5)
        if plus1>=2500:
            target1=500
        elif plus1<=500:
            target1=2500        
        if plus2>=1800:
            target2=1000
        elif plus2<=1000:
            target2=1800

    cv2.putText(frame,"FPS:{0}".format(int(1/(time.time() - start_time))),(250,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(255,0,255),2)
    cv2.imshow("frame", frame) 
    start_time = time.time()
 
    if search == 4:
        if radius >0 and radius<20 :
            change_speed(0.2,0.2)
        else:
            stop()
            search=1
#    print(search)
#    print(' ')
#    print(search1)
    if cv2.waitKey(1) ==27 :
        camera.release() 
        cv2.destroyAllWindows()
        if(plus1>1500):
            for x in range(plus1,1500,-10):
                pi.set_servo_pulsewidth(servopin1,x)
                time.sleep(0.05)
        else:
            for x in range(plus1,1500,10):
                pi.set_servo_pulsewidth(servopin1,x)
                time.sleep(0.05)
 
        if(plus2>1200):
            for x in range(plus2,1200,-10):
                pi.set_servo_pulsewidth(servopin2,x)
                time.sleep(0.05)
        else:
            for x in range(plus2,1200,10):
                pi.set_servo_pulsewidth(servopin2,x)
                time.sleep(0.05)
        break
 
pi.stop()
stop()


'''    elif (ball_x<80)or (ball_x>200):
        #error=150-x  
        #out=abs(error*kp)
        #turn=out+0.5
        if ball_x<80:
            #if radius>25 and ball_x<80:
              #  change_speed(0.5,0.2)
            #else:
                #change_speed(turn,0.5)
            turn_right()
        if ball_x>200:
           # if radius>25 and ball_x<240:
             ##   change_speed(0.2,0.5)
           # else:
              #  change_speed(0.5,turn)
            turn_left()
   elif search==2:
        if radius<20:
            #print("itis%d"%search)
            change_speed(0.2,0.2)

''' 


