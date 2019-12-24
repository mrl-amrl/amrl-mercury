import rospy
from mercury_joy.msg import Joy as MercuryJoy


class Joy:
    def __init__(self, do_register=True, auto_zero=False):
        self._callbacks = {
            'buttons': {},
            'axes': {},
        }
        self._args = {
            'buttons': {},
            'axes': {},
        }
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
        if len(self._callbacks['buttons']) != 0:
            for button in buttons:
                if button in self._callbacks['buttons']:
                    self._callbacks['buttons'][button](
                        *self._args['buttons'][button])

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
        self._callbacks['buttons'][button_name] = callback
        self._args['buttons'][button_name] = args

    def on_changed(self, axes_name, callback, *args):
        self._callbacks['axes'][axes_name] = callback
        self._args['axes'][axes_name] = args
    
    def subscribe(self, callback):
        self._subscribers[id(callback)] = callback

    def unsubscribe(self, name):
        if name in self._callbacks['buttons']:
            del self._callbacks['buttons'][name]
        if name in self._callbacks['axes']:
            del self._callbacks['axes'][name]

        if name in self._args['buttons']:
            del self._args['buttons'][name]
        if name in self._args['axes']:
            del self._args['axes'][name]

    def unsubscribe_all(self):
        self._callbacks = {
            'buttons': {},
            'axes': {},
        }
        self._args = {
            'buttons': {},
            'axes': {},
        }

    def register(self):
        if self.subscriber is not None:
            return
        rospy.logwarn(
            "[{}] registering to /mercury/joy".format(rospy.get_name()))
        self.subscriber=rospy.Subscriber(
            '/mercury/joy', MercuryJoy, self._joy_callback)

    def unregister(self):
        if self.subscriber is None:
            return
        rospy.logwarn(
            "[{}] unregistering from /mercury/joy".format(rospy.get_name()))
        self.subscriber.unregister()
        del self.subscriber
        self.subscriber=None

    @staticmethod
    def spin():
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("mercury_test_joy", anonymous = False)

    def on_pressed():
        rospy.logwarn('pressed')

    def on_changed(value):
        rospy.logwarn(value)

    controller=Joy(auto_zero = True)
    controller.on_pressed('x_button', on_pressed)
    controller.on_changed('left_x_axes', on_changed)
    controller.spin()
