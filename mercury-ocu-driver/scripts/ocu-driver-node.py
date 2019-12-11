#!/usr/bin/env python

import rospy
from controller import JoyController


if __name__ == "__main__":
    rospy.init_node("mercury_ocu_driver", anonymous=False)

    controller = JoyController()    
    controller.spin()
