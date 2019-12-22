import socket


class PowerManagementConnection:
    def __init__(self, target, port=3024):
        self.power_socket = socket.socket(
            family=socket.AF_INET,
            type=socket.SOCK_DGRAM
        )
        self.power_socket.bind(('192.168.10.10', port))
        self.port = port
        self.target = target

        self.epos_reset = 0
        self.led_state = 0
        self.emg_stop = 0
        self.pc = 1
        self.laser = 1
        self.video_server = 1

    def send(self):
        datagram = bytearray()
        datagram.append(self.led_state)
        datagram.append(self.emg_stop)
        datagram.append(self.epos_reset)
        datagram.append(self.pc)
        datagram.append(self.laser)
        datagram.append(self.video_server)
        for _ in range(10):
            datagram.append(0)

        self.power_socket.sendto(datagram, (self.target, self.port))
