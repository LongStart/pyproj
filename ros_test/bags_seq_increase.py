#!/usr/bin/env python

import rospy
import rosbag
from sys import argv


if __name__ == "__main__":
    if(len(argv) < 2):
        print('ex: python merge_bags.py input.bag')
        quit()
    input_bag_name = argv[1]
    output_bag_name = argv[1][:-4] + '_seq.bag'

    output_bag = rosbag.Bag(output_bag_name, 'w')
    input_bag = rosbag.Bag(input_bag_name)
    topic_cnt_dict = {}
    for topic, msg, t in input_bag.read_messages():
        if topic not in topic_cnt_dict:
            topic_cnt_dict[topic] = 0
        else:
            topic_cnt_dict[topic] += 1
        msg.header.seq = topic_cnt_dict[topic]
        output_bag.write(topic, msg, t)
    input_bag.close()
    output_bag.close()