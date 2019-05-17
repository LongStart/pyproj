#!/usr/bin/env python
# coding=utf-8

import rosbag
import sys
import matplotlib.pyplot as plt
import glob
import os

if(len(sys.argv) < 2):
    print("Please input bag file name")
    print("Example: python write_frame_id_to_euroc_position.py file.bag")
bag_path = sys.argv[1]
# bags = glob.glob(glob_input)


with rosbag.Bag(bag_path,'w') as outbag:
    for topic, msg, t in outbag.read_messages():
        if topic == '/leica/position':
            msg.header.frame_id = "position_frame"
            outbag.write(topic, msg, msg.header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
        # print(".")
        # sys.stdout.flush()  


