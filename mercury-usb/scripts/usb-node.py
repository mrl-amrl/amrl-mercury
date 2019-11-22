#!/usr/bin/env python

import rospy
from os import path
from mercury_usb.msg import Devices

if __name__ == "__main__":
    rospy.init_node("mercury_usb", anonymous=False)

    publisher = rospy.Publisher("/mercury/usb", Devices, queue_size=1)
    devices = rospy.get_param("~devices", "").split(",")

    try:
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if publisher.get_num_connections() > 0:
                msg = Devices()
                for device in devices:
                    if path.exists(device):
                        msg.devices.append(device)
                publisher.publish(msg)
            rate.sleep()
    except KeyboardInterrupt:
        rospy.signal_shutdown('keyboard interrupt')
