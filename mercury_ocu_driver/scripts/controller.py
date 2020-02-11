import rospy
import socket
from threading import Thread
from mercury import PowerController
from mercury import logger
from mercury_common.srv import SetEnabled, SetEnabledRequest
from collections import namedtuple
from ocu_command import OCUCommand

PowerTuple = namedtuple('Power', [
    'arm_light', 'arm_laser', 'alarm', 'front_light', 'arm_state', 'emergency'
])


class OCUDriverController:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('192.168.10.10', 10131))

        self.power = PowerController()
        self.last_data = PowerTuple(*([0] * 6))
        self.ocu_command = OCUCommand('192.168.10.8', 9000)

        rospy.Service('/mercury/ocu/led', SetEnabled, self.ocu_led_handler)

    def shutdown(self):
        self.socket.close()

    def ocu_led_handler(self, data):
        self.ocu_command.emergency_led = 0 if data.enabled else 1
        self.ocu_command.send()
        return data.enabled

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                data, _ = self.socket.recvfrom(8)
            except socket.error as err:
                logger.log_warn(err)
                return                        

            data = map(ord, data)
            data = data[:6]
            data = PowerTuple(*data)

            if self.last_data.front_light != data.front_light:
                self.power.send('front_led', data.front_light)
            if self.last_data.emergency != data.emergency:
                self.power.send('emergency', data.emergency)

            self.last_data = data
            rate.sleep()


if __name__ == "__main__":
    ocu = OCUDriverController()
    ocu.spin()
