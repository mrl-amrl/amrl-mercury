import rospy
from mercury_joy.msg import Joy as MercuryJoy


class Joy:
    def __init__(self, do_register=True, auto_zero=False):
        self._callbacks = {
            'buttons': {},
            'axes': {},
            'press_btns': {},
        }
        self._args = {
            'buttons': {},
            'axes': {},
            'press_btns': {},
        }
        self.last_button_states = []
        self.subscriber = None
        self.auto_zero = auto_zero
        if auto_zero:
            self.last_callback_axes = {}
        if do_register:
            self.register()
        self._subscribers = {}

    def _joy_callback(self, data):
        # TODO: mutex
        buttons = data.button_names
        if len(self._callbacks['buttons']) + len(self._callbacks['press_btns']):
            diff_list = list(set(buttons) - set(self.last_button_states))
            for button in buttons:
                if button in self._callbacks['press_btns']:
                    if button in diff_list:
                        self._callbacks['press_btns'][button](
                            *self._args['press_btns'][button])
                if button in self._callbacks['buttons']:
                    self._callbacks['buttons'][button](
                        *self._args['buttons'][button])
            self.last_button_states = buttons

        axes = data.axes_names
        if len(self._callbacks['axes']) != 0:
            if self.auto_zero:
                temporary_lastaxes_callbacks = self.last_callback_axes.copy()
                self.last_callback_axes = {}

            for idx, name in enumerate(axes):
                if name in self._callbacks['axes']:
                    self._callbacks['axes'][name](
                        data.axes_values[idx], *self._args['axes'][name])
                    if self.auto_zero:
                        self.last_callback_axes[name] = data.axes_values[idx]
                        if name in temporary_lastaxes_callbacks:
                            del temporary_lastaxes_callbacks[name]

            if self.auto_zero:
                for name in temporary_lastaxes_callbacks:
                    self._callbacks['axes'][name](0, *self._args['axes'][name])

        for subscriber in self._subscribers.values():
            subscriber(data)

    def on_pressed(self, button_name, callback, *args):
        self._callbacks['press_btns'][button_name] = callback
        self._args['press_btns'][button_name] = args

    def on_key_down(self, button_name, callback, *args):
        self._callbacks['buttons'][button_name] = callback
        self._args['buttons'][button_name] = args

    def on_changed(self, axes_name, callback, *args):
        self._callbacks['axes'][axes_name] = callback
        self._args['axes'][axes_name] = args

    def subscribe(self, callback):
        self._subscribers[id(callback)] = callback

    def unsubscribe(self, name):
        for key in self._callbacks:
            if name in self._callbacks[key]:
                del self._callbacks[key][name]
                del self._args[key][name]

    def unsubscribe_all(self):
        for key in self._callbacks:
            self._callbacks[key].clear()
            self._args[key].clear()
        self._subscribers = {}

    def register(self):
        if self.subscriber is not None:
            return
        self.subscriber = rospy.Subscriber(
            '/mercury/joy', MercuryJoy, self._joy_callback)

    def unregister(self):
        if self.subscriber is None:
            return
        self.subscriber.unregister()
        self.subscriber = None

    @staticmethod
    def spin():
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("mercury_test_joy", anonymous=False)

    def on_pressed():
        rospy.logwarn('pressed')

    def on_changed(value):
        rospy.logwarn(value)

    controller = Joy(auto_zero=True)
    controller.on_pressed('x_button', on_pressed)
    controller.on_changed('left_x_axes', on_changed)
    controller.spin()
