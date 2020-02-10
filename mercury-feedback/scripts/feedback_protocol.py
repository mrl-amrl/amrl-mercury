#!/usr/bin/env python
import socket
from mercury import logger

def low_high_byte(integer):
    low_byte = integer & 0xFF
    high_byte = integer >> 8
    return low_byte, high_byte


def byte_to_variable(low_byte1, byte2, byte3=None, high_byte4=None):
    if byte3 == None and high_byte4 == None:
        return (low_byte1 & 0xFF) + ((byte2 & 0xFF) << 8)
    elif not byte3 == None and high_byte4 == None:
        return (low_byte1 & 0xFF) + (byte2 << 8) + (byte3 << 16)
    elif not byte3 == None and not high_byte4 == None:
        return (low_byte1 & 0xFF) + (byte2 << 8) + (byte3 << 16) + (high_byte4 << 24)


class EposFault():
    def __init__(self):
        self.traction_right = 0
        self.traction_left = 0
        self.front_arm_fault = 0
        self.rear_arm_fault = 0
        self.manip_joint1 = 0
        self.manip_joint2 = 0
        self.manip_joint3 = 0


class EposPosition:
    def __init__(self):
        self.front_arm = 0.0
        self.rear_arm = 0.0
        self.manip_joint1 = 0.0
        self.manip_joint2 = 0.0
        self.manip_joint3 = 0.0
        self.reserve = 0.0


class Battery:
    def __init__(self):
        self.power_battery = 3
        self.signal_battery = 0


class Current:
    def __init__(self):
        self.right_traction = 0
        self.left_traction = 0
        self.manip_joint1 = 0
        self.manip_joint2 = 0
        self.manip_joint3 = 0


class Torque:
    def __init__(self):
        self.right_traction = 0
        self.left_traction = 0
        self.manip_joint1 = 0
        self.manip_joint2 = 0
        self.manip_joint3 = 0


class SensorBoard:
    def __init__(self):
        self.overCurrent_joint4 = 0
        self.overCurrent_joint5 = 0
        self.overCurrent_joint6 = 0
        self.position_joint4 = 0
        self.co2_gas = 0
        self.direction_joint4 = 0


class FeedBackProtocol:
    def __init__(self, reciver_main_board_port=3031, reciver_sensor_board_port=3033):
        self.epos_fault = EposFault()
        self.epos_position = EposPosition()
        self.battery = Battery()
        self.current = Current()
        self.torque = Torque()
        self.sensor_board = SensorBoard()

        self.socket_main_board = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)        
        self.socket_main_board.bind(('0.0.0.0', reciver_main_board_port))
        self.socket_sensor_board = socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM)
        self.socket_sensor_board.bind(('0.0.0.0', reciver_sensor_board_port))

    def deserilise_main_board_data(self):
        try:
            data, _ = self.socket_main_board.recvfrom(1024)
        except socket.error as err:
            logger.log_warn(err)
            return
        logger.log_warn(len(data))
        data_decimal = map(ord, data)        
        self.epos_fault.traction_right = byte_to_variable(
            data_decimal[0], data_decimal[1])
        self.epos_fault.traction_left = byte_to_variable(
            data_decimal[2], data_decimal[3])
        self.epos_fault.front_arm_fault = byte_to_variable(
            data_decimal[4], data_decimal[5])
        self.epos_fault.rear_arm_fault = byte_to_variable(
            data_decimal[6], data_decimal[7])
        self.epos_fault.manip_joint1 = byte_to_variable(
            data_decimal[8], data_decimal[9])
        self.epos_fault.manip_joint2 = byte_to_variable(
            data_decimal[10], data_decimal[11])
        self.epos_fault.manip_joint3 = byte_to_variable(
            data_decimal[12], data_decimal[13])
        self.epos_position.front_arm = data_decimal[14]
        self.epos_position.rear_arm = data_decimal[15]
        self.epos_position.manip_joint1 = byte_to_variable(
            data_decimal[18], data_decimal[19])
        self.epos_position.manip_joint2 = byte_to_variable(
            data_decimal[20], data_decimal[21])
        self.epos_position.manip_joint3 = byte_to_variable(
            data_decimal[22], data_decimal[23])

        self.battery.power_battery = data_decimal[26]
        self.battery.signal_battery = data_decimal[27]

        self.current.right_traction = byte_to_variable(
            data_decimal[28], data_decimal[29])
        self.current.left_traction = byte_to_variable(
            data_decimal[30], data_decimal[31])
        # self.current.manip_joint1 = byte_to_variable(
        #     data_decimal[32], data_decimal[33])
        # self.current.manip_joint2 = byte_to_variable(
        #     data_decimal[34], data_decimal[35])
        # self.current.manip_joint3 = byte_to_variable(
        #     data_decimal[36], data_decimal[37])
        return data_decimal

    def deserilise_sensor_board_data(self):
        try:
            data, _ = self.socket_sensor_board.recvfrom(1024)
        except socket.error as err:
            logger.log_warn(err)
            return
            
        data_decimal = map(ord, data)
        self.sensor_board.overCurrent_joint4 = data_decimal[0]
        self.sensor_board.overCurrent_joint5 = data_decimal[1]
        self.sensor_board.overCurrent_joint6 = data_decimal[2]
        self.sensor_board.position_joint4 = byte_to_variable(
            data_decimal[3], data_decimal[4], data_decimal[5], data_decimal[6])
        self.sensor_board.co2_gas = byte_to_variable(
            data_decimal[7], data_decimal[8])
        self.sensor_board.direction_joint4 = data_decimal[9]
