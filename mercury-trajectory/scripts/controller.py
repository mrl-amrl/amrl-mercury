import rospy

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from mercury_trajectory.cfg import TrajectoryConfig
from mercury import Joy
from socket_connection import SocketConnection


class TrajectoryController:
    def __init__(self):
        self.curve_movement = False
        self.axes_threshold = 0.5
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
        }

        self.joy = Joy()
        # connect to dynamic reconfigure server.
        DynamicReconfigureServer(TrajectoryConfig, self._configuration)
        self.socket = SocketConnection(
            target=rospy.get_param('~board_ip', '192.168.10.16'),
            port=int(rospy.get_param('~board_port', '3020')),
        )

    def axes_change(self, value, name):
        name = name[0]
        rospy.logerr('{} {}'.format(name, value))

    def button_change(self, name):
        rospy.logerr(name)

    def _configuration(self, config, level):
        self.curve_movement = bool(config['curve_movement'])
        self.axes_threshold = float(config['axes_threshold'])
        for key in self.key_items:
            self.key_items[key] = config[key]

        self.joy.unsubscribe_all()
        self.joy.on_changed(
            self.key_items['linear_axes'], self.axes_change, 'linear')
        if self.curve_movement:
            self.joy.on_changed(
                self.key_items['angular_axes'], self.axes_change, 'angular')
        else:
            self.joy.on_pressed(
                self.key_items['btn_turn_left'], self.button_change, 'btn_turn_left')
            self.joy.on_pressed(
                self.key_items['btn_turn_right'], self.button_change, 'btn_turn_right')
            self.joy.on_pressed(
                self.key_items['btn_speed_decr'], self.button_change, 'btn_speed_decr')
            self.joy.on_pressed(
                self.key_items['btn_speed_ecnr'], self.button_change, 'btn_speed_ecnr')
            self.joy.on_pressed(
                self.key_items['btn_front_flipper_up'], self.button_change, 'btn_front_flipper_up')
            self.joy.on_pressed(
                self.key_items['btn_front_flipper_down'], self.button_change, 'btn_front_flipper_down')
            self.joy.on_pressed(
                self.key_items['btn_rear_flipper_up'], self.button_change, 'btn_rear_flipper_up')
            self.joy.on_pressed(
                self.key_items['btn_rear_flipper_down'], self.button_change, 'btn_rear_flipper_down')
        return config

    @staticmethod
    def spin():
        rospy.spin()
