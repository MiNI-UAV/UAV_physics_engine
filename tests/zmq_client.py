import zmq
import time


context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("ipc:///tmp/dron1")
topicfilter = ""
socket.subscribe(topicfilter)

while(1):
    s = socket.recv_string()
    print(s)
