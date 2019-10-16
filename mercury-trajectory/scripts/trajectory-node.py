#!/usr/bin/env python

import rospy
from controller import TrajectoryController


if __name__ == "__main__":
    rospy.init_node("mercury_trajectory", anonymous=False)

    controller = TrajectoryController()
    controller.spin()
