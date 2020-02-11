import rospy
import time

from mercury import Joy, PowerController
from xbox_controller.srv import Rumble
from mercury_common.srv import SetEnabled
from mercury_dynparam.srv import Default
from mercury_state_controller.msg import State
from std_msgs.msg import String


class StateController:
    def __init__(self):
        self.last_time_arm_toggler = time.time()
        self.counter_arm_toggler = 0
        self.power_controller = PowerController()
        self.is_armed = False

        self.joy = Joy()
        self.joy.on_pressed(
            rospy.get_param('~back_button', 'back_button'),
            self.on_pressed
        )
        self.joy.on_key_down(
            rospy.get_param('~arm_toggler', 'left_axes_button right_axes_button'),
            self.arm_toggle,
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
            '/mercury/state_controller/current_state', State, queue_size=10)
        rospy.Service(
            '/mercury/state_controller/change_mode',
            Default,
            self.mode_change_handler,
        )
        rospy.Service(
            '/mercury/state_controller/arm',
            SetEnabled,
            self.arm_handler,
        )
        self.rumble = rospy.ServiceProxy('/mercury/joy/rumble', Rumble)

    def set_arm_state(self, state):
        self.is_armed = state
        self.rumble.call(500)
        if state:
            self.power_controller.send('emergency', False)
            self.services[self.current_state].call(True)
            self.default_loader.call(self.default_keys[self.current_state])
        else:
            self.power_controller.send('emergency', True)
            self.services['manipulator'].call(False)
            self.services['trajectory'].call(False)        
    
    def arm_handler(self, data):
        self.set_arm_state(data.enabled)
        return self.is_armed
    
    def arm_toggle(self):
        current_time = time.time()
        diff_time = (current_time - self.last_time_arm_toggler)
        rate = 1 / diff_time
        if diff_time > 1:
            self.counter_arm_toggler = 0
        else:
            self.counter_arm_toggler += 1
            if self.counter_arm_toggler > rate * 3:
                self.counter_arm_toggler = 0

                self.set_arm_state(not self.is_armed)

                for i in range(6):
                    self.power_controller.send('front_led', i % 2 == 0)
                    time.sleep(0.05)
        self.last_time_arm_toggler = current_time

    def mode_change_handler(self, data):
        if data.name not in self.services:
            return False
        
        for key in self.services:
            if not self.services[key].call(key == data.name):
                return False
        self.current_state = data.name
        return self.default_loader.call(self.default_keys[data.name])

    def on_pressed(self):
        if not self.is_armed:
            return False
        self.services[self.current_state].call(False)
        self.current_state = 'manipulator' if self.current_state == 'trajectory' else 'trajectory'
        self.services[self.current_state].call(True)
        self.default_loader.call(self.default_keys[self.current_state])

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state_publisher.get_num_connections():
                msg = State()
                msg.state = self.current_state
                msg.is_armed = self.is_armed
                self.current_state_publisher.publish(msg)
            rate.sleep()
