import socket
import os 

SOCKET_PATH = "/tmp/munin.sock"


client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

#print("Hello")
client.connect(SOCKET_PATH)

while True:

    try:
        message = "[Important Data, Such data]"

        client.sendall(message.encode("utf-8"))
    except KeyboardInterrupt:
        client.shutdown(socket.SHUT_WR)
        print("Disconnecting from socket")
        break



