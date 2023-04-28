import zmq
import time
from time import sleep


context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://192.168.0.128:9090")
angle = 0.0
while(True):
    socket.send_string(f"pos:0.0,0.0,-5.0,0.0,0.0,{angle}")
    angle += 0.1
    sleep(0.2)
    print("A")