#!/usr/bin/env python

import rospy
from controller import PowerController


if __name__ == "__main__":
    rospy.init_node("mercury_power", anonymous=False)

    controller = PowerController()
    rospy.spin()
