#!/usr/bin/env python

import rospy
from math import degrees, radians, floor, ceil
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

rospy.init_node("mercury_laser_distance", anonymous=False)

publisher = rospy.Publisher('/mercury/laser/distance', Float32, queue_size=10)
target_angle = int(rospy.get_param('~angle', default=0))


def callback(data):
    if publisher.get_num_connections() == 0:
        return
    angle_min = degrees(data.angle_min)
    angle_max = degrees(data.angle_max)
    total_angle = angle_max - angle_min
    degree_part = int(floor(len(data.ranges) / total_angle))
    center_index = int(degree_part * (target_angle - angle_min))
    center = data.ranges[center_index]
    publisher.publish(center)


rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
