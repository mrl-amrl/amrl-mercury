import rospy

from socket_connection import PowerManagementConnection
from mercury_common.srv import SetEnabled


class PowerController:
    def __init__(self):
        for name in ['epos_reset', 'emergency', 'front_led', 'laser', 'pc', 'video_server']:
            rospy.Service(
                "/mercury/power/" + name,
                SetEnabled,
                self.service_handler(name)
            )

        self.power_connection = PowerManagementConnection(
            target=rospy.get_param('~board_ip', '192.168.10.170'),
            port=int(rospy.get_param('~board_port', '3024')),
        )
        self.power_connection.send()

    def service_handler(self, name):
        def handler(data):
            if name == 'epos_reset':
                self.power_connection.epos_reset = data.enabled
            elif name == 'emergency':
                self.power_connection.emg_stop = data.enabled
            elif name == 'front_led':
                self.power_connection.led_state = data.enabled
            elif name == 'laser':
                self.power_connection.laser = data.enabled
            elif name == 'pc':
                self.power_connection.pc = data.enabled
            elif name == 'video_server':
                self.power_connection.video_server = data.enabled
            self.power_connection.send()
            return data.enabled
        return handler
