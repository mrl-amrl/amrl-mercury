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
