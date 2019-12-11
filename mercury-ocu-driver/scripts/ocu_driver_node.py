#!/usr/bin/env python

import rospy
from controller import OCUDriverController


if __name__ == "__main__":
    rospy.init_node("mercury_ocu_driver", anonymous=False)

    controller = OCUDriverController()
    controller.spin()
    rospy.on_shutdown(controller.shutdown)
