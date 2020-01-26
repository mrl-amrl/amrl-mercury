#!/usr/bin/env python
from __future__ import division
import rospy
import socket
import time
import ping
from mercury import logger
from mercury_feedback.msg import Ping


class RobotNetPing:
    def __init__(self):
        self.ping_sub = rospy.Publisher(
            '/feedback/ping', Ping, queue_size=10)

    def spin(self):
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():            
            if self.ping_sub.get_num_connections() == 0:                
                pass
            else:
                msg = Ping()
                t = ping.get_ping_time('192.168.10.170')                
                msg.main_board = int(t) if t > 1 else 1
                t = ping.get_ping_time('192.168.10.20')
                msg.sensor_board = int(t) if t > 1 else 1
                self.ping_sub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node('mercury_netping', anonymous=True)
    try:
        robot_feedback = RobotNetPing()
        robot_feedback.spin()
    except KeyboardInterrupt:
        rospy.signal_shutdown('keyboard interrupt')
    except rospy.ROSException as err:
        logger.log_error("{}".format(err))
