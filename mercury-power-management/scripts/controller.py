import rospy

from mercury import logger
from socket_connection import PowerManagementConnection
from mercury_common.srv import SetEnabled


class PowerController:
    def __init__(self):
        for name in ['epos_reset', 'emergency', 'front_led', 'laser', 'pc', 'video_server', 'manipulator_reset']:
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
    
    def spin(self):
        rospy.spin()
        # rate = rospy.Rate(100)
        # while not rospy.is_shutdown():
        #     self.power_connection.send()
        #     rate.sleep()
    
    def service_handler(self, name):
        def handler(data):           
            value = 1 if data.enabled else 0 
            if name == 'epos_reset':
                self.power_connection.epos_reset = 3 if data.enabled else 0 
            elif name == 'manipulator_reset':
                self.power_connection.epos_reset = 1 if data.enabled else 0 
            elif name == 'emergency':
                self.power_connection.emg_stop = value
            elif name == 'front_led':
                self.power_connection.led_state = value
            elif name == 'laser':
                self.power_connection.laser = value
            elif name == 'pc':
                self.power_connection.pc = value
            elif name == 'video_server':
                self.power_connection.video_server = value
            else:
                return False
            self.power_connection.send()
            return data.enabled
        return handler
