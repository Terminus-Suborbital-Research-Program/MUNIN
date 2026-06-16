import socket
import os 
from typing import Self

class MuninNode:
    """Class representing a Munin node"""
    def __init__(self, _socket_path:str = "/tmp/munin.sock"):
        self.isCommandNode = False
        self.socketPath = _socket_path
        self.socket = None
        self.ipAddress = ""
        self.port = 4455
        self.makeUnixSocket()
        

    def changeUnixSocket(self, socket_path: str) -> Self:
        self.socketPath = socket_path
        self.makeUnixSocket()

    def makeUnixSocket(self) -> Self:
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        return self

    def bindUnixSocket(self) -> Self:
        self.socket.bind(self.socketPath)
    
    def connectUnixSocket(self) -> Self:
        self.socket.connect(self.socketPath)
        return self
    
    def shutdown(self) -> Self:
        self.socket.shutdown(socket.SHUT_WR)
        return self

    def closeSocket(self) -> Self:
        return self

    def changeIpAddress(self, ipAddress: str) -> Self:
        self.ipAddress = ipAddress
        return self

    def changePort(self, port: int) -> Self:
        self.port = port
        return self

    def startup(self) -> Self:
        if self.isCommandNode:
            pass
        return self

    def sendRecordingCommand():
        raise NotImplementedError()


    def sendTargetCommand():
        raise NotImplementedError()

    def sendTargetCoordinates(self, message: str) -> Self:
        self.socket.sendall( message.encode("utf-8"))
        return self