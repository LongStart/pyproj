#!/usr/bin/env python

import rospy
import rosbag
from sys import argv


if __name__ == "__main__":
    if(len(argv) < 2):
        print('ex: python merge_bags.py output_name.bag in1.bag in2.bag ')
        quit()
    output_bag_name = 'merged_' + str(len(argv) - 1) + '.bag'
    output_bag = rosbag.Bag(output_bag_name, 'w')
    for bag_name in argv[1:]:
        bag = rosbag.Bag(bag_name)
        for topic, msg, t in bag.read_messages():
            output_bag.write(topic, msg, t)
        bag.close()
    output_bag.close()