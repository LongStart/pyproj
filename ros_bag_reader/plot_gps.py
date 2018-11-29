#!/usr/bin/env python
# coding=utf-8

import rosbag
import sys
import matplotlib.pyplot as plt
import glob
import os

if(len(sys.argv) < 2):
    print("Please input bag file name")
    print("Example: python plot_gps.py bag_path")
glob_input = sys.argv[1] + "/*.bag"
bags = glob.glob(glob_input)


for bag_path in bags:
    gps_poses = []
    pos_x = []
    pos_y = []
    bag = rosbag.Bag(bag_path)
    for topic, msg, t in bag.read_messages(topics=['/vehicle/seriesGPSout']):
        gps_poses += [msg]
        pos_x += [msg.latitude]
        pos_y += [msg.longitude]
    bag.close()
    plt.plot(pos_x, pos_y, 'o-')
    print(".")
    sys.stdout.flush()


plt.axis('equal')
plt.show()

