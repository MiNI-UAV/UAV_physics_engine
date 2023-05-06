import zmq
import time


context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://localhost:9090")
topicfilter = "pos"
socket.subscribe(topicfilter)

while(1):
    s = socket.recv_string()
    print(s)
