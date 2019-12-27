import socket
import rospy
from mercury_power_management.msg import States


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
        if not rospy.is_shutdown():
            self.publisher = rospy.Publisher('/mercury/power_management/states', States, queue_size=10)

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

        if not rospy.is_shutdown():
            if self.publisher.get_num_connections():
                msg = States()
                msg.epos_reset = self.epos_reset == 1
                msg.led_state = self.led_state == 1
                msg.emg_stop = self.emg_stop == 1
                msg.pc = self.pc == 1
                msg.laser = self.laser == 1
                msg.video_server = self.video_server == 1
                self.publisher.publish(msg)
        self.power_socket.sendto(datagram, (self.target, self.port))
