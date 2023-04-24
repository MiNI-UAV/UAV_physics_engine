import zmq
import time
from time import sleep


context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("ipc:///tmp/drone1/control")
counter = 0
for _ in range(3):
    socket.send_string("c:ping")
    sleep(0.5)
socket.send_string("w:5.0,6.0,7.0")
socket.send_string("s:1.0,2.0,3.0,4.0")
sleep(2)
for _ in range(3):
    socket.send_string("c:start")
    sleep(1)
    socket.send_string("c:pause")
    sleep(1)
socket.send_string("c:stop")