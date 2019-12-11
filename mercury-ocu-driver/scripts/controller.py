import rospy
import socket
from mercury import PowerController
from mercury import logger
from mercury_common.srv import SetEnabled, SetEnabledRequest
from collections import namedtuple

PowerTuple = namedtuple('Power', [
    'arm_light', 'arm_laser', 'alarm', 'front_light', 'arm_state', 'emergency'
])

class ArmEnableService:
    def __init__(self):
        pass
    
    def send(self, state):
        proxy = rospy.ServiceProxy('/mercury/trajectory/arm_enable', SetEnabled)    
        request = SetEnabledRequest()
        request.enabled = state            
        response = proxy.call(request)
        return response.state

class OCUDriverController:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('192.168.10.10', 10131))

        self.power = PowerController()
        self.arm_controller = ArmEnableService()

        self.last_data = PowerTuple(*([0] * 6))

    def shutdown(self):
        self.socket.close()

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
                if data.emergency:
                    self.arm_controller.send(False)

            self.last_data = data            
            rate.sleep()


if __name__ == "__main__":
    ocu = OCUDriverController()
    ocu.spin()
