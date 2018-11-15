import fcntl
import sys
import os
import time
import tty
import termios
import socket

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#sock.setblocking(0)
# Connect the socket to the port on the server given by the caller
hosts='192.168.0.107'
server_address = (hosts, 8888)
sock.connect(server_address)

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
                sock.sendall(c.encode())
#                data = sock.recv(1600).decode()
                print(repr(c))
            except IOError:
                error='no'
                endall(error.encode())
                print('not ready')
            time.sleep(.035)
sock.close()