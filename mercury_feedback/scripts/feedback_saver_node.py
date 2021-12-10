#!/usr/bin/env python
# author amir sharifi

import rospy
from mercury_common.msg import RobotsFeedback, MovementFeedback, ManipulatorStatus
from matplotlib import pyplot as plt
from datetime import datetime
import itertools
import os
import pandas as pd

node_name = 'robot_feedback_analysis'


class FeedBackAnalysis:
    def __init__(self, robot_feedback_topic, movement_feedback_topic, manipulator_feedback_topic):
        rospy.Subscriber(robot_feedback_topic, RobotsFeedback,
                         self._robot_feedback_cb)
        rospy.Subscriber(movement_feedback_topic,
                         MovementFeedback, self._movement_feedback_cb)
        rospy.Subscriber(manipulator_feedback_topic,
                         ManipulatorStatus, self._manipulator_feedback_cb)

        self.start_time = rospy.get_time()
        self.robot_currents = []
        self.robot_torque = []
        self.duration = []

        self.current = {'current_right': [],
                        'current_left': [],
                        'current_sum': [],
                        'current_front_arm': [],
                        'current_rear_arm': [],
                        'current_joint0': [],
                        'current_joint1': [],
                        'current_joint2': [],
                        'time': []}

        self.torque = {
            'torque_right': [],
            'torque_left': [],
            'torque_rear_arm': [],
            'torque_front_arm': [],
            'torque_joint0': [],
            'torque_joint1': [],
            'torque_joint2': [],
            'time': []
        }

        self.manipulator_positions = {
            'joint0': [],
            'joint1': [],
            'joint2': [],
            'time': []
        }

        self.movement_position = {
            'rear_arm': [],
            'front_arm': [],
            'time': []
        }

        self.movement_rpm = {
            'left': [],
            'right': [],
            'time': []
        }

    def _manipulator_feedback_cb(self, manipulator_msg):
        self.manipulator_positions['joint0'].append(
            manipulator_msg.position[0])
        self.manipulator_positions['joint1'].append(
            manipulator_msg.position[1])
        self.manipulator_positions['joint2'].append(
            manipulator_msg.position[2])

    def _movement_feedback_cb(self, movement_msg):
        self.movement_position['front_arm'].append(movement_msg.position[0])
        self.movement_position['rear_arm'].append(movement_msg.position[1])

        self.movement_rpm['left'].append(movement_msg.left_rpm)
        self.movement_rpm['right'].append(movement_msg.right_rpm)

    def _robot_feedback_cb(self, robotfeedback):
        """
        get karo_feedBack topic and set sum important param
        @param current dic{key:value}
        @param key of current dic : current right,current left,current joint0,current joint1,current joint2,current joint3
        @param key of torque_dic : torque right,torque left,torque joint0,torque joint1,torque joint2,torque joint3
        """
        secs_start = rospy.get_time() - self.start_time
        self.current['current_right'].append(robotfeedback.current_right)
        self.current['current_left'].append(robotfeedback.current_left)
        self.current['current_sum'].append(
            robotfeedback.current_left + robotfeedback.current_right)
        self.current['current_front_arm'].append(
            robotfeedback.current_front_arm)
        self.current['current_rear_arm'].append(robotfeedback.current_rear_arm)
        self.current['current_joint0'].append(robotfeedback.current_joint0)
        self.current['current_joint1'].append(robotfeedback.current_joint1)
        self.current['current_joint2'].append(robotfeedback.current_joint2)

        self.torque['torque_right'].append(robotfeedback.torque_right)
        self.torque['torque_left'].append(robotfeedback.torque_left)
        self.torque['torque_rear_arm'].append(robotfeedback.torque_rear_arm)
        self.torque['torque_front_arm'].append(robotfeedback.torque_front_arm)
        self.torque['torque_joint0'].append(robotfeedback.torque_joint0)
        self.torque['torque_joint1'].append(robotfeedback.torque_joint1)
        self.torque['torque_joint2'].append(robotfeedback.torque_joint2)

        self.duration.append(secs_start)

    def save_data(self, file_name):
        self.current['time'] = self.duration
        self.torque['time'] = self.duration
        self.manipulator_positions['time'] = self.duration
        self.movement_position['time'] = self.duration
        self.movement_rpm['time'] = self.duration
        current_df = pd.DataFrame(self.current)
        torque_df = pd.DataFrame(self.torque)
        movement_df = pd.DataFrame(self.movement_position)
        movement_rpm_df = pd.DataFrame(self.movement_rpm)
        manipulator_df = pd.DataFrame(self.manipulator_positions)
        current_df.to_csv(os.path.join(file_name, 'current.csv'))
        torque_df.to_csv(os.path.join(file_name, 'torque.csv'))
        movement_df.to_csv(os.path.join(file_name, 'movement.csv'))
        movement_rpm_df.to_csv(os.path.join(file_name, 'movement_rpm.csv'))
        manipulator_df.to_csv(os.path.join(file_name, 'manipulator.csv'))

    @staticmethod
    def spin():
        rospy.spin()

def main():
    robot_feedback_topic = rospy.get_param(
        node_name + '/robot_feedback_topic', '/feedback/robot')
    movement_feedback_topic = rospy.get_param(
        node_name + '/movement_feedback_topic', '/feedback/movement')
    manipulator_feedback_topic = rospy.get_param(
        node_name + '/manipulator_feedback_topic', '/feedback/manipulator')

    feed_back_analysis = FeedBackAnalysis(robot_feedback_topic,movement_feedback_topic,manipulator_feedback_topic)
    path_file = rospy.get_param(
        node_name + '/path_to_save_data', '/home/mrl/catkin_ws/src/amrl-mercury/mercury_feedback/data')
    
    exel_is_enable = rospy.get_param(node_name + '/exel_is_enable', True)
    plot_is_enable = rospy.get_param(node_name + '/plot_is_enable', True)
    plot_mode = rospy.get_param(node_name + '/plot_mode', 'all')
    date = datetime.now()
    dir_name = date.strftime("%Y-%m-%d-%H:%M")
    os.mkdir(os.path.join(path_file, dir_name))

    dir_path = os.path.join(path_file, dir_name)
    try:
        feed_back_analysis.spin()
        rospy.signal_shutdown(node_name + ' is Shoting down wait for plt')

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
    finally:
        if exel_is_enable:
            feed_back_analysis.save_data(dir_path)




if __name__ == "__main__":
    rospy.init_node(node_name, anonymous=True)
    main()
