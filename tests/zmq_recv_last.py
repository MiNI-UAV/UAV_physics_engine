import zmq
import time


context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.set(zmq.CONFLATE,1)
socket.connect("tcp://127.0.0.1:9090")
topicfilter = ""
socket.subscribe(topicfilter)

while(1):
    s = socket.recv_string()
    print(s)
