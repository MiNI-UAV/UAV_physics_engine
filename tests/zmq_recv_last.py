import zmq
import time


context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.set(zmq.CONFLATE,1)
socket.connect("ipc:///tmp/dron1/state")
topicfilter = ""
socket.subscribe(topicfilter)

while(1):
    s = socket.recv_string()
    print(s)
    time.sleep(1)
