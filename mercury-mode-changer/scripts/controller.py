import rospy
from mercury import Joy
from mercury_common.srv import SetEnabled
from mercury_dynparam.srv import Default
from std_msgs.msg import String


class ModeChanger:
    def __init__(self):
        self.joy = Joy()
        self.joy.on_pressed(
            rospy.get_param('~back_button', 'back_button'),
            self.on_pressed
        )
        self.current_state = 'trajectory'
        self.default_keys = {
            'trajectory': 'trajectory_joy',
            'manipulator': 'manipulator_joy',
        }
        self.services = {
            'trajectory': rospy.ServiceProxy(
                '/mercury/trajectory/enable',
                SetEnabled
            ),
            'manipulator': rospy.ServiceProxy(
                '/mercury/manipulator/enable',
                SetEnabled
            ),
        }
        self.default_loader = rospy.ServiceProxy(
            '/mercury/dynparam/defaults',
            Default
        )
        self.current_state_publisher = rospy.Publisher(
            '/mercury/mode_changer/current_state', String, queue_size=10)
        self.mode_changer_service = rospy.Service(
            '/mercury/mode_changer/change',
            Default,
            self.mode_change_handler,
        )

    def mode_change_handler(self, data):
        if data.name not in self.services:
            return False
        
        for key in self.services:
            if not self.services[key].call(key == data.name):
                return False
        return self.default_loader.call(self.default_keys[data.name])

    def on_pressed(self):
        self.services[self.current_state].call(False)
        self.current_state = 'manipulator' if self.current_state == 'trajectory' else 'trajectory'
        self.services[self.current_state].call(True)
        self.default_loader.call(self.default_keys[self.current_state])

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state_publisher.get_num_connections():
                msg = String()
                msg.data = self.current_state
                self.current_state_publisher.publish(msg)
            rate.sleep()
