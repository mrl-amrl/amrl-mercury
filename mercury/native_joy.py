import rospy
from sensor_msgs.msg import Joy


class NativeJoyCallback:
    def __init__(self, axis=False, button=False):
        self._selector = 'axis' if axis else 'button'
        self._callbacks = []
        self._filters = {
            'reverse': False,
            'factor': 1,
            'axis_to_button_threshold': None,
        }

    def listen(self, callback, *args):
        self._callbacks.append((callback, args))
        return self

    def clear(self):
        self._callbacks = []
        return self

    def filter(self, reverse=False, factor=1, axis_to_button_threshold=None):
        self._filters['reverse'] = reverse
        self._filters['factor'] = factor
        self._filters['axis_to_button_threshold'] = axis_to_button_threshold
        return self

    def _filter(self, value):
        if self._filters['reverse']:
            if self._selector == 'axis':
                value = -value
            else:
                value = 1 - value
        value *= self._filters['factor']
        if self._filters['axis_to_button_threshold'] is not None:
            if value > self._filters['axis_to_button_threshold']:
                value = 1
            elif value < -1 * self._filters['axis_to_button_threshold']:
                value = -1
            else:
                value = 0
        return value

    def call(self, value):
        if isinstance(value, list):
            for i in range(len(value)):
                value[i] = self._filter(value[i])
        else:
            value = self._filter(value)
        for callback, args in self._callbacks:
            callback(value, *args)


class NativeJoy:
    def __init__(self):
        self._callbacks = {
            'buttons': {},
            'axes': {},
        }
        rospy.Subscriber('/joy', Joy, self._joy_callback)

    def clear(self):
        for callback in self._callbacks['buttons'].values():
            del callback
        for callback in self._callbacks['axes'].values():
            del callback
        self._callbacks = {
            'buttons': {},
            'axes': {},
        }

    def _joy_callback(self, data):
        axes = data.axes
        buttons = data.buttons
        for button_keys, button_callback in self._callbacks['buttons'].items():
            button_keys = map(int, button_keys.split(','))
            should_call = True
            for button_key in button_keys:
                if not buttons[button_key]:
                    should_call = False
                    break
            if should_call:
                button_callback.call(1)
        for axis_keys, (axis_callback, threshold) in self._callbacks['axes'].items():
            axis_keys = map(int, axis_keys.split(','))
            should_call = True
            values = []
            for axis_key in axis_keys:
                if abs(axes[axis_key]) < threshold:
                    should_call = False
                    break
                else:
                    values.append(axes[axis_key])
            if should_call:
                axis_callback.call(values)

    def on(self, axes=None, buttons=None, threshold=0.05):
        if axes == None and buttons == None:
            raise Exception("Axes or Buttons is not valid")

        if buttons != None:
            callback = NativeJoyCallback(button=True)
            self._callbacks['buttons'][",".join(map(str, buttons))] = callback
            return callback

        if axes != None:
            callback = NativeJoyCallback(axis=True)
            self._callbacks['axes'][",".join(map(str, axes))] = (
                callback, threshold)
            return callback

    @staticmethod
    def spin():
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("mercury_test_joy", anonymous=False)

    def on_move(value, result):
        rospy.logwarn([value, result])

    controller = NativeJoy()
    controller.on(axes=[0, 1], threshold=-1).filter(reverse=True,
                                                    factor=2).listen(on_move, 'salam')
    controller.spin()
