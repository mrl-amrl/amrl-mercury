import rospy
from mercury import logger

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from mercury_joy.cfg import JoyConfig
from mercury_joy.msg import Joy as MercuryJoy
from sensor_msgs.msg import Joy as ROSJoy


class JoyController:
    def __init__(self):
        self.button_names = []
        self.axes_names = []
        self.axes_to_button = []
        self.button_to_axes = []
        self.last_button_to_axes_states = {}
        self.axes_threshold = 0.025
        self.map_axes = []
        self.first_run = {
            2: True,
            5: True,
        }

        self.axes_button_trigger = {}

        # connect to dynamic reconfigure server.
        DynamicReconfigureServer(JoyConfig, self.configuration)

        # subscribe to ROS-joy topic.
        rospy.Subscriber(
            rospy.get_param('~joy_topic', default='/joy'),
            ROSJoy,
            self.callback,
            queue_size=int(rospy.get_param('~joy_queue_size', default='10')),
        )

        # mercury-joy publisher.
        self.publisher = rospy.Publisher(
            '/mercury/joy',
            MercuryJoy,
            queue_size=int(rospy.get_param(
                '~mercury_queue_size',
                default='1'
            ))
        )

        # register a callback for ros shutdown
        rospy.on_shutdown(self.shutdown)

        self.message = MercuryJoy()

    def configuration(self, config, level):
        self.axes_button_trigger = {}

        # get button_names from dynamic-reconfigure server
        button_names = str(config['button_names'])
        self.button_names = [button_name.strip()
                             for button_name in button_names.split(',')]

        # get axes_names from dynamic-reconfigure server
        axes_names = str(config['axes_names'])
        self.axes_names = [axes_name.strip()
                           for axes_name in axes_names.split(',')]

        # get axes_names from dynamic-reconfigure server
        button_to_axes = str(config['button_to_axes'])
        self.button_to_axes = [button_name.strip()
                               for button_name in button_to_axes.split(',')]
        self.last_button_to_axes_states = {}

        # validate and extract axes_to_button config
        # if result is None it means that configuration string
        # is bad.
        axes_to_button = str(config['axes_to_button'])
        if len(axes_to_button) > 2:
            axes_to_button = self._extract_axes_to_button_config(
                axes_to_button)
            if not axes_to_button:
                rospy.logerr('bad axes_to_button configuration.')
            else:
                # the structure of each item is like:
                # {
                #   'axes_idx': <number>,
                #   'mode': <'less-than', 'greater-than'>,
                #   'threshold': <number>,
                #   'button_name': <string>
                # }
                self.axes_to_button = axes_to_button
        else:
            self.axes_to_button = []

        # validate and extract map_axes config, like axes_to_button
        map_axes = self._extract_map_axes_config(
            str(config['map_axes']))
        if not map_axes:
            rospy.logerr('bad map_axes configuration.')
        else:
            self.map_axes = map_axes
        return config

    def callback(self, data):
        # return if no one subscribe to mercury-joy publisher.
        if self.publisher.get_num_connections() == 0:
            return

        mercury_joy = MercuryJoy()
        mercury_joy.header.stamp = rospy.Time.now()
        mercury_joy.button_names = []
        mercury_joy.axes_names = []
        mercury_joy.axes_values = []

        axes = list(data.axes)

        for key in self.first_run:
            if not self.first_run[key]:
                continue
            if axes[key] == 0.0:
                axes[key] = 1.0
            else:
                self.first_run[key] = False

        for idx, button in enumerate(data.buttons):
            if idx < len(self.button_names):
                if self.button_names[idx] in self.button_to_axes:
                    if idx in self.last_button_to_axes_states:
                        last_state = self.last_button_to_axes_states[idx]
                        if last_state != button:
                            mercury_joy.axes_names.append(
                                self.button_names[idx])
                            mercury_joy.axes_values.append(button)
                    else:
                        self.last_button_to_axes_states[idx] = button
                        mercury_joy.axes_names.append(self.button_names[idx])
                        mercury_joy.axes_values.append(button)
                elif button:
                    mercury_joy.button_names.append(self.button_names[idx])

        for idx, axes_value in enumerate(axes):
            for map_axes_item in self.map_axes:
                if map_axes_item['axes_idx'] == idx:
                    # TODO: default value of joy when ROS-joy node started is equal to 0.0.
                    # mercury will detect it pressed with value of 1.0!
                    axes_value = self._map_axes_value(
                        axes_value, map_axes_item)
                    break

            is_on_axes_to_buttons = False
            for axes_to_button in self.axes_to_button:
                if axes_to_button['axes_idx'] == idx:
                    is_on_axes_to_buttons = True
                    if axes_to_button['mode'] == 'less-than' and axes_value < axes_to_button['threshold']:
                        mercury_joy.button_names.append(
                            axes_to_button['button_name']
                        )
                    if axes_to_button['mode'] == 'greater-than' and axes_value > axes_to_button['threshold']:
                        mercury_joy.button_names.append(
                            axes_to_button['button_name']
                        )
            if is_on_axes_to_buttons:
                continue

            if abs(axes_value) < self.axes_threshold:
                continue
            if idx < len(self.axes_names):
                mercury_joy.axes_names.append(self.axes_names[idx])
                mercury_joy.axes_values.append(axes_value)

        self.message = mercury_joy
        # self.publisher.publish(self.message)

    def spin(self):
        rate = rospy.Rate(int(rospy.get_param(
            '~mercury_publish_rate',
            default='30'
        )))
        while not rospy.is_shutdown():
            if self.publisher.get_num_connections() > 0:
                self.publisher.publish(self.message)
            rate.sleep()

    def shutdown(self, reason='unknown'):
        rospy.signal_shutdown(reason)

    def _map_axes_value(self, value, map_item):
        left_span = map_item['from_end'] - map_item['from_start']
        rigth_span = map_item['to_end'] - map_item['to_start']
        scale = float(value - map_item['from_start']) / float(left_span)
        return map_item['to_start'] + (scale * rigth_span)

    def _extract_map_axes_config(self, cfg):
        items = []
        start_index = 0
        closed_paran = True
        for i, char in enumerate(cfg):
            if char == '(':
                start_index = i + 1
                if not closed_paran:
                    return None
                closed_paran = False
            if char == ')':
                closed_paran = True
                if i < start_index:
                    return None
                section = cfg[start_index:i]
                parts = section.split(',')
                if len(parts) != 5:
                    return None
                if not parts[0].isdigit():
                    return None
                items.append({
                    'axes_idx': int(parts[0]),
                    'from_start': float(parts[1]),
                    'from_end': float(parts[2]),
                    'to_start': float(parts[3]),
                    'to_end': float(parts[4]),
                })
        return items

    def _extract_axes_to_button_config(self, cfg):
        items = []
        start_index = 0
        closed_paran = True
        for i, char in enumerate(cfg):
            if char == '(':
                start_index = i + 1
                if not closed_paran:
                    return None
                closed_paran = False
            if char == ')':
                closed_paran = True
                if i < start_index:
                    return None
                section = cfg[start_index:i]
                parts = section.split(',')
                if len(parts) != 4:
                    return None
                if not parts[0].isdigit():
                    return None
                if parts[1] not in ['less-than', 'greater-than']:
                    return None
                items.append({
                    'axes_idx': int(parts[0]),
                    'mode': parts[1],
                    'threshold': float(parts[2]),
                    'button_name': parts[3],
                })
        return items
