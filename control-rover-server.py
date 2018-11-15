

import sys
import time
import os
import fcntl
import time
import tty
import termios
import socket
import RPi.GPIO as GPIO


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the address given on the command line
hosts='192.168.0.106'
server_address = (hosts, 8888)
print ('starting up on %s port %s' % server_address)
sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
sock.bind(server_address)
sock.listen(1)



IN1=11
IN2=12
IN3=13
IN4=15
IN5=16
IN6=18

GPIO.setwarnings(False)

def init():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(IN1,GPIO.OUT)
    GPIO.setup(IN2,GPIO.OUT)
    GPIO.setup(IN3,GPIO.OUT)
    GPIO.setup(IN4,GPIO.OUT)
    GPIO.setup(IN5,GPIO.OUT)
    GPIO.setup(IN6,GPIO.OUT)
   
def forward():
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)
#    p=GPIO.PWM(IN5,50)
#    q=GPIO.PWM(IN6,50)   
#    p.start(100)
#    q.start(100)
 #   time.sleep(sleep_time)
#    GPIO.cleanup()   
   
def back():
    GPIO.output(IN1,GPIO.LOW)
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.LOW)
    GPIO.output(IN4,GPIO.HIGH)
#    p=GPIO.PWM(IN5,50)
#    q=GPIO.PWM(IN6,50)   
#    p.start(100)
#    q.start(100)
#    time.sleep(sleep_time)
#    GPIO.cleanup()

def left():
    GPIO.output(IN1,False)
    GPIO.output(IN2,False)
    GPIO.output(IN3,GPIO.HIGH)
    GPIO.output(IN4,GPIO.LOW)
#    q=GPIO.PWM(IN6,50)
#   q.start(100)
#    time.sleep(sleep_time)
#    GPIO.cleanup()

def right():
    GPIO.output(IN1,GPIO.HIGH)
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,False)
    GPIO.output(IN4,False)
#    p=GPIO.PWM(IN5,50)
#    p.start(10)
#    for dc in range(10,101,5):
#        p.ChangeDutyCycle(dc)
#        time.sleep(0.5)
    #time.sleep(sleep_time)
#    p.stop()
#    GPIO.cleanup()

def stop():
    GPIO.output(IN1,False)
    GPIO.output(IN2,False)
    GPIO.output(IN3,False)
    GPIO.output(IN4,False)
    GPIO.cleanup()
   
GPIO.cleanup()   
while True:
    print ('waiting for a connection')
    connection,client_address = sock.accept()
    print ('client connected:',client_address)
    sock.setblocking(0)
    while True:
        try:
            c = connection.recv(5).decode()
            init()
            if c == 'w':
                print(repr(c))
                forward()
            elif c== 's':
                print(repr(c))
                back()
            elif c == 'a':
                print(repr(c))
                left()
            elif c == 'd':
                print(repr(c))
                right()
            else:
                stop()
#                break
        except socket.error:
            print("nor ready")
            stop()
connection.close()
