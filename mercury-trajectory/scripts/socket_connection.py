from PySide2.QtNetwork import QUdpSocket, QHostAddress
from PySide2.QtCore import QByteArray, QDataStream, QIODevice, QObject

class SocketConnection(QObject):
    def __init__(self, target, port=3020):
        super(SocketConnection, self).__init__(None)
        self.socket = QUdpSocket(self)
        self.socket.bind(QHostAddress(QHostAddress.Any), port)
        self.target = QHostAddress(target)

    def send(self, 
        left_velocity, 
        right_velocity, 
        arm_mode, 
        arm_front_direction, 
        arm_rear_direction,
    ):
        datagram = QByteArray()
        stream = QDataStream(datagram, QIODevice.WriteOnly)
        
        dir_left = 0
        dir_right = 0
        if left_velocity > 0:
            dir_left = 1
        elif left_velocity < 0:
            dir_left = 2
        if right_velocity > 0:
            dir_right = 1
        elif right_velocity < 0:
            dir_right = 2
        
        left_velocity = abs(left_velocity)
        right_velocity = abs(right_velocity)        
        stream.writeUInt8(dir_left)
        stream.writeUInt8(dir_right)
        stream.writeUInt8(left_velocity)
        stream.writeUInt8(right_velocity)
        stream.writeUInt8(arm_mode)
        stream.writeUInt8(arm_front_direction)
        stream.writeUInt8(arm_rear_direction)
        self.socket.writeDatagram(datagram, self.target, 3020)