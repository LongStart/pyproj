#!/usr/bin/env python
# coding=utf-8

import rosbag
import rospy
import sys

from geometry_msgs.msg import PointStamped

if(len(sys.argv) < 2):
    print("Please input bag file name")
    print("Example: python creat_bag.py file.bag")
bag_path = sys.argv[1]

bag = rosbag.Bag(bag_path, 'w')

try:
    position = PointStamped
    # position.header.stamp = rospy.get_rostime()
    position.point.x = 1

    bag.write('/leica/position', position)
finally:
    bag.close()


