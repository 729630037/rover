import cv2
import sys
import os
import fcntl
import time
import tty
import sys
import time
import os
import fcntl
import time
import tty
import termios
import socket
import RPi.GPIO as GPIO
import serial    #import serial module
ser = serial.Serial('/dev/ttyUSB1', 9600,timeout=1);   #open named port at 9600,1s timeot

#try and exceptstructure are exception handler
class raw(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()
    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)
    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)

class nonblocking(object):
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()
    def __enter__(self):
        self.orig_fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.orig_fl | os.O_NONBLOCK)
    def __exit__(self, *args):
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.orig_fl)

with raw(sys.stdin):
    with nonblocking(sys.stdin):
        while True:
            try:
                c = sys.stdin.read(1)
                ser.write(c);#writ a string to port
                response = ser.readall();#read a string from port
                print(response)
            except IOError:
                pass
#                print('not ready')
        time.sleep(.035)
ser.close()