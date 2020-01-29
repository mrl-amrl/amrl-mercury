import socket
from mercury import logger


class MovementConnection:
    def __init__(self, target, port=3020):
        self.movement_socket = socket.socket(
            family=socket.AF_INET,
            type=socket.SOCK_DGRAM
        )
        self.movement_socket.bind(('0.0.0.0', port))
        self.port = port
        self.target = target

    def send(self,
             left_velocity=0,
             right_velocity=0,
             arm_front_direction=0,
             arm_rear_direction=0,
             ):
        datagram = bytearray()

        if left_velocity > 100:
            left_velocity = 100
        elif left_velocity < -100:
            left_velocity = -100
        if right_velocity > 100:
            right_velocity = 100
        elif right_velocity < -100:
            right_velocity = -100

        dir_left = 0x00
        dir_right = 0x00
        if left_velocity > 0:
            dir_left = 0x01
        elif left_velocity < 0:
            dir_left = 0x02
        if right_velocity > 0:
            dir_right = 0x01
        elif right_velocity < 0:
            dir_right = 0x02

        left_velocity = abs(left_velocity)
        right_velocity = abs(right_velocity)
        datagram.append(dir_left)
        datagram.append(dir_right)
        datagram.append(chr(int(left_velocity)))
        datagram.append(chr(int(right_velocity)))
        datagram.append(0)
        datagram.append(arm_front_direction)
        datagram.append(arm_rear_direction)
        datagram.append(0xFF)
        self.movement_socket.sendto(datagram, (self.target, self.port))
