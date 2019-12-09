import rospy

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from socket_connection import MovementConnection, PowerManagementConnection
from mercury_trajectory.cfg import TrajectoryConfig
from mercury_trajectory.msg import TrajectoryData
from mercury_common.srv import SetEnabled
from mercury import Joy


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
        }

        self.is_armed = False

        rospy.Service("/mercury/trajectory/arm_enable",
                      SetEnabled, self._serive_callback)
        rospy.Service("/mercury/power/front_led",
                      SetEnabled, self._service_power_fled_handler)
        rospy.Service("/mercury/power/arm_led",
                      SetEnabled, self._service_power_aled_handler)
        rospy.Service("/mercury/power/laser",
                      SetEnabled, self._service_power_laser_handler)
        rospy.Service("/mercury/power/pc",
                      SetEnabled, self._service_power_pc_handler)
        rospy.Service("/mercury/power/video_server",
                      SetEnabled, self._service_power_video_server_handler)

        self.joy = Joy()
        # connect to dynamic reconfigure server.
        DynamicReconfigureServer(TrajectoryConfig, self._configuration)
        self.power_connection = PowerManagementConnection(
            target=rospy.get_param('~board_ip', '192.168.10.170'),
            port=int(rospy.get_param('~board_port', '3024')),
        )
        self.movement_connection = MovementConnection(
            target=rospy.get_param('~board_ip', '192.168.10.170'),
            port=int(rospy.get_param('~board_port', '3020')),
        )
        self.publisher = rospy.Publisher(
            '/mercury/trajectory_raw', TrajectoryData, queue_size=10)

        self.power_connection.led_state = 0
        self.power_connection.emg_stop = 0        
        self.power_connection.send()
        self.commands = self.new_message()

    def _service_power_video_server_handler(self, data):
        self.power_connection.video_server = data.enabled
        rospy.logwarn("[mercury-trajectory] sevice called for 'video server' with {}, sending ...".format(data.enabled))
        self.power_connection.send()
        return data.enabled
    
    def _service_power_pc_handler(self, data):
        self.power_connection.pc = data.enabled
        rospy.logwarn("[mercury-trajectory] sevice called for 'pc' with {}, sending ...".format(data.enabled))
        self.power_connection.send()
        return data.enabled
    
    def _service_power_laser_handler(self, data):
        self.power_connection.laser = data.enabled
        rospy.logwarn("[mercury-trajectory] sevice called for 'laser' with {}, sending ...".format(data.enabled))
        self.power_connection.send()
        return data.enabled

    def _service_power_fled_handler(self, data):
        self.power_connection.led_state = data.enabled
        rospy.logwarn("[mercury-trajectory] sevice called for 'fled' with {}, sending ...".format(data.enabled))
        self.power_connection.send()
        return data.enabled

    def _service_power_aled_handler(self, data):
        rospy.logwarn("[mercury-trajectory] sevice called for 'aled' with {}, sending ...".format(data.enabled))
        return data.enabled

    def _serive_callback(self, data):        
        self.is_armed = data.enabled
        self.power_connection.emg_stop = 0 if self.is_armed else 1
        rospy.logwarn("[mercury-trajectory] sevice called for 'emg' with {}, sending ...".format(data.enabled))
        self.power_connection.send()
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

    def send(self):    
        left_velocity = self.commands['linear'] * \
            self.max_speed - self.commands['angular'] * self.max_speed
        right_velocity = self.commands['linear'] * \
            self.max_speed + self.commands['angular'] * self.max_speed
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
            msg.angular = self.commands['angular'] * self.max_speed
            msg.linear = self.commands['linear'] * self.max_speed
            msg.arm_front = self.commands['arm_front_direction']
            msg.arm_rear = self.commands['arm_rear_direction']
            msg.left = left_velocity
            msg.right = right_velocity
            msg.armed = self.is_armed
            self.publisher.publish(msg)
        
        if self.is_armed:
            self.movement_connection.send(
                left_velocity=left_velocity,
                right_velocity=right_velocity,
                arm_front_direction=self.commands['arm_front_direction'],
                arm_rear_direction=self.commands['arm_rear_direction'],
            )

    def new_message(self):
        return {
            'linear': 0,
            'angular': 0,
            'arm_front_direction': 0,
            'arm_rear_direction': 0,
        }

    def _configuration(self, config, level):
        rospy.logwarn("[mercury-trajectory] new dynamic parameters has been loaded")
        self.curve_movement = bool(config['curve_movement'])
        self.axes_threshold = float(config['axes_threshold'])
        for key in self.key_items:
            self.key_items[key] = config[key]

        self.joy.unsubscribe_all()
        self.joy.on_changed(
            self.key_items['linear_axes'],
            self.axes_change,
            'linear_axes'
        )
        self.joy.on_pressed(
            self.key_items['btn_front_flipper_up'],
            self.button_change,
            'btn_front_flipper_up'
        )
        self.joy.on_pressed(
            self.key_items['btn_front_flipper_down'],
            self.button_change,
            'btn_front_flipper_down'
        )
        self.joy.on_pressed(
            self.key_items['btn_rear_flipper_up'],
            self.button_change,
            'btn_rear_flipper_up'
        )
        self.joy.on_pressed(
            self.key_items['btn_rear_flipper_down'],
            self.button_change,
            'btn_rear_flipper_down'
        )

        self.joy.on_pressed(
            self.key_items['btn_both_flipper_up'],
            self.button_change,
            'btn_both_flipper_up'
        )
        self.joy.on_pressed(
            self.key_items['btn_both_flipper_down'],
            self.button_change,
            'btn_both_flipper_down'
        )

        self.joy.on_pressed(
            self.key_items['btn_speed_ecnr'],
            self.button_change,
            'btn_speed_ecnr'
        )
        self.joy.on_pressed(
            self.key_items['btn_speed_decr'],
            self.button_change,
            'btn_speed_decr'
        )        
        self.joy.on_pressed(
            self.key_items['btn_turn_left'],
            self.button_change,
            'btn_turn_left'
        )
        self.joy.on_pressed(
            self.key_items['btn_turn_right'],
            self.button_change,
            'btn_turn_right'
        )
        if self.curve_movement:
            self.joy.on_changed(
                self.key_items['angular_axes'],
                self.axes_change,
                'angular_axes'
            )
        return config

    def spin(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.send()
            self.commands = self.new_message()
            rate.sleep()
