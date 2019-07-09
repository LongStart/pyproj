#!/usr/bin/env python

import rospy
import rosbag
from sys import argv


if __name__ == "__main__":
    if(len(argv) < 2):
        print('ex: python downsample_bag.py input.bag topic_name rate')
        quit()
    input_bagname = argv[1]
    input_topicname = argv[2]
    rate = int(argv[3])
    output_bagname = input_bagname[:-4] + '_downsampled.bag'
    output_bag = rosbag.Bag(output_bagname, 'w')
    input_bag = rosbag.Bag(input_bagname)
    count = 0
    for topic, msg, t in input_bag.read_messages(topics=[input_topicname]):
        if(count % rate == 0):
            output_bag.write(topic, msg, t)
        count += 1
    input_bag.close()
    output_bag.close()