from mercury import logger
from PySide2.QtNetwork import QUdpSocket, QHostAddress
from PySide2.QtCore import QByteArray, QDataStream, QIODevice, QObject

class OCUCommand(QObject):
    def __init__(self, target, port=9000):
        super(OCUCommand, self).__init__(None)
        self.socket = QUdpSocket(self)
        self.socket.bind(QHostAddress('192.168.10.10'), port)
        self.port = port
        self.target = QHostAddress(target)

        self.emergency_led = 0
    
    def send(self):
        datagram = QByteArray()
        stream = QDataStream(datagram, QIODevice.WriteOnly)

        stream.writeUInt8(self.emergency_led)
        self.socket.writeDatagram(datagram, self.target, self.port)
