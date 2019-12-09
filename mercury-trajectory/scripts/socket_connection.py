from PySide2.QtNetwork import QUdpSocket, QHostAddress
from PySide2.QtCore import QByteArray, QDataStream, QIODevice, QObject

class PowerManagementConnection(QObject):
    def __init__(self, target, port=3024):
        super(PowerManagementConnection, self).__init__(None)
        self.socket = QUdpSocket(self)
        self.socket.bind(QHostAddress('192.168.10.10'), port)
        self.port = port
        self.target = QHostAddress(target)

        self.epos_reset = 0
        self.led_state = 0
        self.emg_stop = 0
        self.pc = 1
        self.laser = 1
        self.video_server = 1
    
    def send(self):
        datagram = QByteArray()
        stream = QDataStream(datagram, QIODevice.WriteOnly)

        stream.writeUInt8(self.led_state)
        stream.writeUInt8(self.emg_stop)
        stream.writeUInt8(self.epos_reset)
        stream.writeUInt8(self.pc)
        stream.writeUInt8(self.laser)
        stream.writeUInt8(self.video_server)
        for _ in range(10):
            stream.writeUInt8(0)
        
        self.socket.writeDatagram(datagram, self.target, self.port)

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


if __name__ == "__main__":
    import time
    power_connection = PowerManagementConnection(target='192.168.10.170', port=3024)
    movement_connection = MovementConnection(target='192.168.10.170', port=3020)        
    
    power_connection.send()
    for i in range(0, 100):
        movement_connection.send(
            left_velocity=i,
            right_velocity=i,            
            arm_front_direction=0,
            arm_rear_direction=0,
        )
        time.sleep(0.05)
    for i in range(100, 0, -1):
        movement_connection.send(
            left_velocity=i,
            right_velocity=i,            
            arm_front_direction=0,
            arm_rear_direction=0,
        )
        time.sleep(0.05)
    
