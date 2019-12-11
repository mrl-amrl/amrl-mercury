from mercury import logger

from PySide2.QtNetwork import QUdpSocket, QHostAddress
from PySide2.QtCore import QByteArray, QDataStream, QIODevice, QObject

class MovementConnection(QObject):
    def __init__(self, target, port=3020):
        super(MovementConnection, self).__init__(None)
        self.socket = QUdpSocket(self)
        self.socket.bind(QHostAddress('192.168.10.10'), port)
        self.port = port
        self.target = QHostAddress(target)

    def send(self,
             left_velocity=0,
             right_velocity=0,             
             arm_front_direction=0,
             arm_rear_direction=0,
             ):
        logger.log_error("sending data {} {}".format(left_velocity, right_velocity))
        datagram = QByteArray()
        stream = QDataStream(datagram, QIODevice.WriteOnly)

        if left_velocity > 100:
            left_velocity = 100
        elif left_velocity < -100:
            left_velocity = -100
        if right_velocity > 100:
            right_velocity = 100
        elif right_velocity < -100:
            right_velocity = -100

        dir_left = 0x00
        dir_right = 0x00
        if left_velocity > 0:
            dir_left = 0x01
        elif left_velocity < 0:
            dir_left = 0x02
        if right_velocity > 0:
            dir_right = 0x01
        elif right_velocity < 0:
            dir_right = 0x02

        left_velocity = abs(left_velocity)
        right_velocity = abs(right_velocity)
        stream.writeUInt8(dir_left)
        stream.writeUInt8(dir_right)
        stream.writeUInt8(left_velocity)
        stream.writeUInt8(right_velocity)
        stream.writeUInt8(0)
        stream.writeUInt8(arm_front_direction)
        stream.writeUInt8(arm_rear_direction)
        stream.writeUInt8(0xFF)
        self.socket.writeDatagram(datagram, self.target, self.port)
