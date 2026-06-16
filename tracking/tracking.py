import os 
#import astropy
import muninNode as muninNode

SOCKET_PATH = "/tmp/munin.sock"

#astropy.coordinates
node = muninNode.MuninNode(SOCKET_PATH)
node.connectUnixSocket()

while True:
    try:
        node.sendTargetCoordinates("12.0 23.0")
    except KeyboardInterrupt:
        node.shutdown()
        print("Disconnecting from socket")
        break


#client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
#
##print("Hello")
#client.connect(SOCKET_PATH)
#
#while True:
#
 #   try:
 #       message = "[Important Data, Such data]"
#
 #       client.sendall(message.encode("utf-8"))
 #   except KeyboardInterrupt:
 #       client.shutdown(socket.SHUT_WR)
 #       print("Disconnecting from socket")
 #       break



