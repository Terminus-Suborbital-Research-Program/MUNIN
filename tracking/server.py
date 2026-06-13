import socket
import os 

SOCKET_PATH = "/tmp/munin.sock"

if os.path.exists(SOCKET_PATH):
    os.unlink(SOCKET_PATH)

server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)

t = ["Apple","ere"]
y = t.__str__() 
print(t.__str__())

#print("Hello")
server.bind(SOCKET_PATH)
server.listen(1)

try: 
    while True:
        print("F")
        connection, address = server.accept()
        try:
            print("Client Connected.")
            data = connection.recv(1024)
            if data:
                print(f"Recieved: {data.decode("utf-8")}\n")
                connection.sendall(b"Data")
        finally:
            print("no")
            connection.close()
except KeyboardInterrupt:
    print("g")
finally:
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)

class MuninNode:
    """Class representing a Munin node"""
    def __init__(self):
        self.isCommandNode = False
        self.socketPath = ""
        self.ipAddress = ""
        self.port = 4455

    def changeUnixSocket(self, socket_path: str):
        self.socketPath = socket_path

    def changeUnixSocket(self, ipAddress: str):
        self.ipAddress= ipAddress

    def changePort(self, port: int):
        self.port = port

    def startup(self):
        if self.isCommandNode:
            pass

    def sendRecordingCommand():
        pass

    def sendTargetCommand():
        pass

    def sendTargetCoordinates():
        pass