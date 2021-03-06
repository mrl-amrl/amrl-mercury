import rospy
import time

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from socket_connection import MovementConnection
from mercury_trajectory.cfg import TrajectoryConfig
from mercury_trajectory.msg import TrajectoryData
from mercury_common.srv import SetEnabled
from mercury import logger
from mercury import Joy, PowerController


class TrajectoryController:
    def __init__(self):
        self.curve_movement = False
        self.axes_threshold = 0.5
        self.max_speed = 0
        self.key_items = {
            'linear_axes': 'linear_axes',
            'angular_axes': 'angular_axes',
            'btn_turn_left': 'btn_turn_left',
            'btn_turn_right': 'btn_turn_right',
            'btn_speed_decr': 'btn_speed_decr',
            'btn_speed_ecnr': 'btn_speed_ecnr',
            'btn_front_flipper_up': 'btn_front_flipper_up',
            'btn_front_flipper_down': 'btn_front_flipper_down',
            'btn_rear_flipper_up': 'btn_rear_flipper_up',
            'btn_rear_flipper_down': 'btn_rear_flipper_down',
            'btn_both_flipper_up': 'btn_both_flipper_up',
            'btn_both_flipper_down': 'btn_both_flipper_down',
            'btn_epos_reset': 'btn_epos_reset',
            'btn_front_led': 'btn_front_led',
        }
        self.angular_speed_factor = float(rospy.get_param('angular_speed_factor', '1.0'))
        self.linear_speed_factor = float(rospy.get_param('linear_speed_factor', '1.0'))

        self.enable = False
        rospy.Service("/mercury/trajectory/enable",
                      SetEnabled, self._enable_service)

        self.joy = Joy(do_register=False)
        self.power_controller = PowerController()

        # connect to dynamic reconfigure server.
        DynamicReconfigureServer(TrajectoryConfig, self._configuration)
        self.movement_connection = MovementConnection(
            target=rospy.get_param('~board_ip', '192.168.10.170'),
            port=int(rospy.get_param('~board_port', '3020')),
        )
        self.publisher = rospy.Publisher(
            '/mercury/trajectory_raw',
            TrajectoryData,
            queue_size=10
        )
        self.commands = self.new_message()
        self.current_led_state = False
        self.power_controller.send('front_led', self.current_led_state)

    def _enable_service(self, data):
        logger.log_warn(
            "sevice called for 'enable' with {}".format(data.enabled))
        if data.enabled:
            self.joy.register()
        else:
            self.joy.unregister()
        self.enable = data.enabled
        return data.enabled

    def axes_change(self, value, name):
        if name == 'linear_axes':
            self.commands['linear'] = value
        elif name == 'angular_axes':
            self.commands['angular'] = value

    def button_change(self, name):
        if name == 'btn_both_flipper_up':
            self.commands['arm_front_direction'] = 2
            self.commands['arm_rear_direction'] = 2
        elif name == 'btn_both_flipper_down':
            self.commands['arm_front_direction'] = 1
            self.commands['arm_rear_direction'] = 1
        else:
            if name == 'btn_front_flipper_up':
                self.commands['arm_front_direction'] = 1
            elif name == 'btn_front_flipper_down':
                self.commands['arm_front_direction'] = 2

            if name == 'btn_rear_flipper_up':
                self.commands['arm_rear_direction'] = 1
            elif name == 'btn_rear_flipper_down':
                self.commands['arm_rear_direction'] = 2

        if name == 'btn_speed_ecnr':
            self.max_speed += 1
            if self.max_speed > 100:
                self.max_speed = 100
        elif name == 'btn_speed_decr':
            self.max_speed -= 1
            if self.max_speed < 0:
                self.max_speed = 0

        if name == 'btn_turn_right':
            self.commands['angular'] = -1
        elif name == 'btn_turn_left':
            self.commands['angular'] = 1
        
        if name == 'btn_front_led':
            logger.log_warn(name)
            self.current_led_state = not self.current_led_state
            self.power_controller.send('front_led', self.current_led_state)

    def send(self):
        angular_speed = int(self.max_speed * self.angular_speed_factor)
        linear_speed = int(self.max_speed * self.linear_speed_factor)
        left_velocity = self.commands['linear'] * \
            linear_speed - self.commands['angular'] * angular_speed
        right_velocity = self.commands['linear'] * \
            linear_speed + self.commands['angular'] * angular_speed
        if left_velocity > 100:
            left_velocity = 100
        elif left_velocity < -100:
            left_velocity = -100
        if right_velocity > 100:
            right_velocity = 100
        elif right_velocity < -100:
            right_velocity = -100

        if self.publisher.get_num_connections() > 0:
            msg = TrajectoryData()
            msg.speed = self.max_speed
            msg.angular = self.commands['angular'] * angular_speed
            msg.linear = self.commands['linear'] * linear_speed
            msg.arm_front = self.commands['arm_front_direction']
            msg.arm_rear = self.commands['arm_rear_direction']
            msg.left = left_velocity
            msg.right = right_velocity
            self.publisher.publish(msg)

        self.movement_connection.send(
            left_velocity=left_velocity,
            right_velocity=right_velocity,
            arm_front_direction=self.commands['arm_front_direction'],
            arm_rear_direction=self.commands['arm_rear_direction'],
        )

    def epos_reset(self):
        self.power_controller.send('epos_reset', True)
        time.sleep(0.025)
        self.power_controller.send('epos_reset', False)

    def new_message(self):
        return {
            'linear': 0,
            'angular': 0,
            'arm_front_direction': 0,
            'arm_rear_direction': 0,
        }

    def register(self):
        self.joy.unsubscribe_all()
        self.joy.on_changed(
            self.key_items['linear_axes'],
            self.axes_change,
            'linear_axes'
        )
        self.joy.on_key_down(
            self.key_items['btn_front_flipper_up'],
            self.button_change,
            'btn_front_flipper_up'
        )
        self.joy.on_key_down(
            self.key_items['btn_front_flipper_down'],
            self.button_change,
            'btn_front_flipper_down'
        )
        self.joy.on_key_down(
            self.key_items['btn_rear_flipper_up'],
            self.button_change,
            'btn_rear_flipper_up'
        )
        self.joy.on_key_down(
            self.key_items['btn_rear_flipper_down'],
            self.button_change,
            'btn_rear_flipper_down'
        )

        self.joy.on_key_down(
            self.key_items['btn_both_flipper_up'],
            self.button_change,
            'btn_both_flipper_up'
        )
        self.joy.on_key_down(
            self.key_items['btn_both_flipper_down'],
            self.button_change,
            'btn_both_flipper_down'
        )

        self.joy.on_key_down(
            self.key_items['btn_speed_ecnr'],
            self.button_change,
            'btn_speed_ecnr'
        )
        self.joy.on_key_down(
            self.key_items['btn_speed_decr'],
            self.button_change,
            'btn_speed_decr'
        )
        self.joy.on_key_down(
            self.key_items['btn_turn_left'],
            self.button_change,
            'btn_turn_left'
        )
        self.joy.on_pressed(
            self.key_items['btn_front_led'],
            self.button_change,
            'btn_front_led'
        )
        self.joy.on_key_down(
            self.key_items['btn_turn_right'],
            self.button_change,
            'btn_turn_right'
        )
        self.joy.on_pressed(
            self.key_items['btn_epos_reset'],
            self.epos_reset,
        )
        if self.curve_movement:
            self.joy.on_changed(
                self.key_items['angular_axes'],
                self.axes_change,
                'angular_axes'
            )

    def _configuration(self, config, level):
        logger.log_warn(
            "new dynamic parameters has been loaded")
        self.curve_movement = bool(config['curve_movement'])
        self.axes_threshold = float(config['axes_threshold'])
        for key in self.key_items:
            self.key_items[key] = config[key]

        self.register()
        return config

    def spin(self):
        rate = rospy.Rate(30)
        is_stop = True
        while not rospy.is_shutdown():
            if self.enable:
                self.send()
                self.commands = self.new_message()
                is_stop = False
            elif not is_stop:
                self.commands = self.new_message()
                for _ in range(5):
                    self.send()
                is_stop = True
            rate.sleep()
