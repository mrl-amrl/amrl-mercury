#!/usr/bin/env python
from __future__ import division
import rospy
import socket
import time
import ping
import threading
from timeout import timeout
from std_msgs.msg import UInt8
from mercury_feedback.msg import ManipulatorStatus, MovementFeedback, EposError, RobotsFeedback
from std_msgs.msg import Int16
from feedback_protocol import FeedBackProtocol

node_name = 'robot_feedback'


class MRLRobotFeedBack:
    def __init__(self):
        main_port, sensor_port, queue_size = self.get_params()
        self.feedback_protocol = FeedBackProtocol(main_port, sensor_port)
        self.ping_sub = rospy.Publisher(
            '/feedback/ping', Int16, queue_size=queue_size)
        self.movement_pub = rospy.Publisher(
            '/feedback/movement', MovementFeedback, queue_size=queue_size)
        self.manipulator_pub = rospy.Publisher(
            '/feedback/manipulator', ManipulatorStatus, queue_size=queue_size)
        self.epos_error_pub = rospy.Publisher(
            '/feedback/epos', EposError, queue_size=queue_size)
        self.robot_feedback_pub = rospy.Publisher(
            '/feedback/robot', RobotsFeedback, queue_size=queue_size)
        self.feedback_co2 = rospy.Publisher(
            '/feedback/co2', Int16, queue_size=queue_size)
        self.main_board = rospy.get_param('main_board_available', True)
        self.sensor_board = rospy.get_param('sensor_board_available', False)
        self.exit_thread = False

    def sensor_board_feedback(self):
        if self.feedback_co2.get_num_connections() > 0:
            self.feedback_co2.publish(
                self.feedback_protocol.sensor_board.co2_gas)

    def movement_publisher(self):
        movement_msg = MovementFeedback()
        movement_msg.position.append(
            90 - self.feedback_protocol.epos_position.front_arm
        )
        movement_msg.position.append(
            90 - self.feedback_protocol.epos_position.rear_arm
        )
        if self.movement_pub.get_num_connections() > 0:
            self.movement_pub.publish(movement_msg)

    def manipulator_publisher(self):
        manipulator_status_msg = ManipulatorStatus()

        manipulator_status_msg.name.append('joint1')
        manipulator_status_msg.position.append(
            -self.feedback_protocol.epos_position.manip_joint1 * 300/4096)

        manipulator_status_msg.name.append('joint2')
        manipulator_status_msg.position.append(
            -self.feedback_protocol.epos_position.manip_joint2 * 300/4096)

        manipulator_status_msg.name.append('joint3')
        manipulator_status_msg.position.append(
            -self.feedback_protocol.epos_position.manip_joint3 * 300/4096)

        manipulator_status_msg.name.append('joint4')
        manipulator_status_msg.position.append(
            self.feedback_protocol.sensor_board.position_joint4)
        if self.manipulator_pub.get_num_connections() > 0:
            self.manipulator_pub.publish(manipulator_status_msg)

    def epos_error_publisher(self):
        epos_error_msg = EposError()
        epos_error_msg.name.extend(['traction right error',
                                    'traction left error', 'front arm error',
                                    'rear arm error', 'manipulator joint 1 error',
                                    'manipulator joint 2 error', 'manipulator joint 3 error'])
        fault = []
        fault.append(self.feedback_protocol.epos_fault.traction_right)
        fault.append(self.feedback_protocol.epos_fault.traction_left)
        fault.append(self.feedback_protocol.epos_fault.front_arm_fault)
        fault.append(self.feedback_protocol.epos_fault.rear_arm_fault)
        fault.append(self.feedback_protocol.epos_fault.manip_joint1)
        fault.append(self.feedback_protocol.epos_fault.manip_joint2)
        fault.append(self.feedback_protocol.epos_fault.manip_joint3)
        epos_error_msg.error.extend(fault)
        epos_error_msg.error_description.extend(map(self.error_parser, fault))
        if self.epos_error_pub.get_num_connections() > 0:
            self.epos_error_pub.publish(epos_error_msg)

    def robot_feedback_publisher(self):
        robot_feedback_msg = RobotsFeedback()
        robot_feedback_msg.signal_battery.data = self.feedback_protocol.battery.signal_battery
        robot_feedback_msg.power_battery.data = self.feedback_protocol.battery.power_battery
        robot_feedback_msg.current_right = self.feedback_protocol.current.right_traction
        robot_feedback_msg.current_left = self.feedback_protocol.current.left_traction
        robot_feedback_msg.current_sum = self.feedback_protocol.current.left_traction + \
            self.feedback_protocol.current.right_traction
        robot_feedback_msg.current_joint0 = self.feedback_protocol.current.manip_joint1
        robot_feedback_msg.current_joint1 = self.feedback_protocol.current.manip_joint2
        robot_feedback_msg.current_joint2 = self.feedback_protocol.current.manip_joint3
        robot_feedback_msg.torque_right = self.feedback_protocol.torque.right_traction
        robot_feedback_msg.torque_left = self.feedback_protocol.torque.left_traction
        robot_feedback_msg.torque_joint0 = self.feedback_protocol.torque.manip_joint1
        robot_feedback_msg.torque_joint1 = self.feedback_protocol.torque.manip_joint2
        robot_feedback_msg.torque_joint2 = self.feedback_protocol.torque.manip_joint3
        if self.robot_feedback_pub.get_num_connections() > 0:
            self.robot_feedback_pub.publish(robot_feedback_msg)

    def get_params(self):
        reciver_main_board_port = rospy.get_param(
            '~reciver_main_board_port', 3031)
        reciver_sensor_board_port = rospy.get_param(
            '~reciver_sensor_board_port', 3033)
        queue_size = int(rospy.get_param('~queue_size', 10))
        return reciver_main_board_port, reciver_sensor_board_port, queue_size

    @staticmethod
    def error_parser(fault):
        if fault == 0x0000:
            return "No Error"
        elif fault == 0x1000:
            return "Generic Error"
        elif fault == 0x1080:
            return "Generic initialization error"
        elif fault == 0x1081:
            return "Generic initialization error"
        elif fault == 0x1082:
            return "Generic initialization error"
        elif fault == 0x1083:
            return "Generic initialization error"
        elif fault == 0x1090:
            return "Firmware incompatibility error"
        elif fault == 0x2310:
            return "Over error"
        elif fault == 0x2320:
            return "Power stage protection error"
        elif fault == 0x3210:
            return "Over Voltage"
        elif fault == 0x3220:
            return "Under Voltage"
        elif fault == 0x4210:
            return "Thermal overload error"
        elif fault == 0x5113:
            return "Logic supply voltage too low error"
        elif fault == 0x5280:
            return "Hardware defect error"
        elif fault == 0x5281:
            return "Hardware incompatibility error"
        elif fault == 0x5480:
            return "Hardware error"
        elif fault == 0x5481:
            return "Hardware error"
        elif fault == 0x5482:
            return "Hardware error"
        elif fault == 0x5483:
            return "Hardware error"
        elif fault == 0x6080:
            return "Sign of life error"
        elif fault == 0x6081:
            return "Extension 1 watchdog error"
        elif fault == 0x6180:
            return "Internal software error"
        elif fault == 0x6181:
            return "Internal software error"
        elif fault == 0x6182:
            return "Internal software error"
        elif fault == 0x6183:
            return "Internal software error"
        elif fault == 0x6184:
            return "Internal software error"
        elif fault == 0x6185:
            return "Internal software error"
        elif fault == 0x6186:
            return "Internal software error"
        elif fault == 0x6187:
            return "Internal software error"
        elif fault == 0x6188:
            return "Internal software error"
        elif fault == 0x6189:
            return "Internal software error"
        elif fault == 0x6190:
            return "Internal software error"
        elif fault == 0x6320:
            return "Software parameter error"
        elif fault == 0x7320:
            return "Position sensor error"
        elif fault == 0x7380:
            return "Position sensor breach error"
        elif fault == 0x7381:
            return "Position sensor resolution error"
        elif fault == 0x7382:
            return "Position sensor index error"
        elif fault == 0x7388:
            return "Hall sensor error"
        elif fault == 0x7389:
            return "Hall sensor not found error"
        elif fault == 0x738A:
            return "Hall angle detection error"
        elif fault == 0x738C:
            return "SSI sensor error"
        elif fault == 0x7390:
            return "Missing main sensor error"
        elif fault == 0x7391:
            return "Missing commutation sensor error"
        elif fault == 0x7392:
            return "Main sensor direction error"
        elif fault == 0x8110:
            return "CAN overrun error (object lost)"
        elif fault == 0x8111:
            return "CAN overrun error"
        elif fault == 0x8120:
            return "CAN passive mode error"
        elif fault == 0x8130:
            return "CAN heartbeat error"
        elif fault == 0x8150:
            return "CAN PDO COB-ID collision"
        elif fault == 0x8180:
            return "EtherCAT communication error"
        elif fault == 0x8181:
            return "EtherCAT initialization error"
        elif fault == 0x81FD:
            return "CAN bus turned off"
        elif fault == 0x81FE:
            return "CAN Rx queue overflow"
        elif fault == 0x81FF:
            return "CAN Tx queue overflow"
        elif fault == 0x8210:
            return "CAN PDO length error"
        elif fault == 0x8250:
            return "RPDO timeout"
        elif fault == 0x8280:
            return "EtherCAT PDO communication error"
        elif fault == 0x8281:
            return "EtherCAT SDO communication error"
        elif fault == 0x8611:
            return "Following error"
        elif fault == 0x8A80:
            return "Negative limit switch error"
        elif fault == 0x8A81:
            return "Positive limit switch error"
        elif fault == 0x8A82:
            return "Software position limit error"
        elif fault == 0x8A88:
            return "STO error"
        elif fault == 0xFF01:
            return "System overloaded error"
        elif fault == 0xFF02:
            return "Watchdog error"
        elif fault == 0xFF0B:
            return "System peak overloaded error"
        elif fault == 0xFF10:
            return "Controller gain error"
        elif fault == 0xFF12:
            return "Auto tuning current limit error"
        elif fault == 0xFF13:
            return "Auto tuning identification current error"
        elif fault == 0xFF14:
            return "Auto tuning buffer overflow error"
        elif fault == 0xFF15:
            return "Auto tuning sample mismatch error"
        elif fault == 0xFF16:
            return "Auto tuning parameter error"
        elif fault == 0xFF17:
            return "Auto tuning amplitude mismatch error"
        elif fault == 0xFF18:
            return "Auto tuning period length error"
        elif fault == 0xFF19:
            return "Auto tuning timeout error"
        elif fault == 0xFF20:
            return "Auto tuning standstill error"
        elif fault == 0xFF21:
            return "Auto tuning torque invalid error"
        return "Unknown error"

    def calculate_ping(self):
        rate = rospy.Rate(2)
        while not self.exit_thread:
            time_result = ping.get_ping_time('192.168.10.170')
            if time_result < 1:
                time_result = 1
            else:
                time_result = int(time_result)
            if self.ping_sub.get_num_connections() > 0:
                self.ping_sub.publish(time_result)
            rate.sleep()

    def spin(self):
        thread = threading.Thread(target=self.calculate_ping)
        thread.daemon = True
        thread.start()

        while not rospy.is_shutdown():
            if self.main_board:
                self.feedback_protocol.deserilise_main_board_data()
            if self.sensor_board:
                self.feedback_protocol.deserilise_sensor_board_data()

            self.movement_publisher()
            self.manipulator_publisher()
            self.robot_feedback_publisher()
            self.sensor_board_feedback()
            self.epos_error_publisher()

        self.exit_thread = True


if __name__ == "__main__":
    rospy.init_node(node_name, anonymous=True)
    try:
        robot_feedback = MRLRobotFeedBack()
        robot_feedback.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('keyboard interrupt')
    except rospy.ROSException as err:
        rospy.logerr("[mercury-feedback] {}".format(err))
