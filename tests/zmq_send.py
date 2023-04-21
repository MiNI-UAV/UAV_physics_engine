import zmq
import time
from time import sleep


context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect("ipc:///tmp/dron1/control")
counter = 0
while (True):
    socket.send_string(f"Wind: {counter}")
    counter+= 1
    sleep(1)