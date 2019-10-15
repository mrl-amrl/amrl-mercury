import rospy
from mercury_joy.msg import Joy as MercuryJoy


class Joy:
    def __init__(self):
        self._callbacks = {
            'buttons': {},
            'axes': {},
        }
        rospy.Subscriber('/mercury/joy', MercuryJoy, self._joy_callback)

    def _joy_callback(self, data):
        buttons = data.button_names
        if len(self._callbacks['buttons']) != 0:
            for button in buttons:
                if button in self._callbacks['buttons']:
                    self._callbacks['buttons'][button]()

        axes = data.axes_names
        if len(self._callbacks['axes']) != 0:
            for idx, name in enumerate(axes):
                if name in self._callbacks['axes']:
                    self._callbacks['axes'][name](data.axes_values[idx])
    
    def on_pressed(self, button_name, callback):
        self._callbacks['buttons'][button_name] = callback
    
    def on_changed(self, axes_name, callback):
        self._callbacks['axes'][axes_name] = callback

    def unsubscribe(self, name):
        if name in self._callbacks['buttons']:
            del self._callbacks['buttons'][name]
        if name in self._callbacks['axes']:
            del self._callbacks['axes'][name]

    @staticmethod
    def spin():
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("mercury_test_joy", anonymous=False)

    def on_pressed():
        rospy.logwarn('pressed')
    
    def on_changed(value):
        rospy.logwarn(value)

    controller = Joy()
    controller.on_pressed('x_button', on_pressed)
    controller.on_changed('left_x_axes', on_changed)
    controller.spin()
