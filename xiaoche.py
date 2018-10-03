import sys
import time
import os
import fcntl
import time
import tty
import termios
import socket
import RPi.GPIO as GPIO
import gpiozero as gz


sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the address given on the command line
hosts='192.168.0.110'
server_address = (hosts, 8888)
print ('starting up on %s port %s' % server_address)
sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
sock.bind(server_address)
sock.listen(1)



motor_r = gz.Motor(17,18)
motor_l = gz.Motor(27,22)

def forward():
    motor_r.forward()
    motor_l.forward()
    
def back():
    motor_r.backward()
    motor_l.backward()
    
def left():
    motor_r.forward()
    
def right():
    motor_l.forward()

def ro():
    motor_r.forward()
    motor_l.backward()
    
def lo():
    motor_l.forward()
    motor_r.backward()
    
def stop():
    motor_r.stop()
    motor_l.stop()
 
while True:
    print ('waiting for a connection')
    connection,client_address = sock.accept()
    print ('client connected:',client_address)
    while True:
        try:
            c = connection.recv(5).decode()
#            init()
            if c == 'w':
                print(repr(c))
                forward()
            elif c == 's':
                print(repr(c))
                back()
            elif c == 'a':
                print(repr(c))
                left()
            elif c == 'd':
                print(repr(c))
                right()
            elif c == 'q':
                print(repr(c))
                ro()
            elif c == 'e':
                print(repr(c))
                lo()            
            else:
                stop()
#                break
        except socket.error:
            print("nor ready")
            stop()
connection.close()