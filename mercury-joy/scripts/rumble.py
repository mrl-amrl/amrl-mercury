import time
from evdev import ecodes, ff, InputDevice


class RumbleController:
    def __init__(self, device_path):
        self.device = InputDevice(device_path)

    def do(self, duration_ms, repeat_count=1):
        rumble = ff.Rumble(strong_magnitude=0xc000, weak_magnitude=0xc000)

        effect = ff.Effect(
            ecodes.FF_RUMBLE,  # type
            -1,  # id (set by ioctl)
            0,  # direction
            ff.Trigger(0, 0),  # no triggers
            ff.Replay(duration_ms, 0),  # length and delay
            ff.EffectType(ff_rumble_effect=rumble)
        )

        effect_id = self.device.upload_effect(effect)
        self.device.write(ecodes.EV_FF, effect_id, repeat_count)
        time.sleep((duration_ms / 1000.0) * repeat_count)
        self.device.erase_effect(effect_id)
