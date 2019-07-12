#!/usr/bin/env python

import rospy
import rosbag
from sys import argv


if __name__ == "__main__":
    if(len(argv) < 4):
        print('ex: python downsample_bag.py input.bag rate[10] topic_name[/images/image_raw] ')
        quit()
    input_bagname = argv[1]
    rate = int(argv[2])
    input_topicnames = [argv[i] for i in range(3, len(argv))]
    
    output_bagname = input_bagname[:-4] + '_downsampled.bag'
    output_bag = rosbag.Bag(output_bagname, 'w')
    input_bag = rosbag.Bag(input_bagname)
    for input_topicname in input_topicnames:
        count = 0
        for topic, msg, t in input_bag.read_messages(topics=[input_topicname]):
            if(count % rate == 0):
                output_bag.write(topic, msg, t)
            count += 1
    input_bag.close()
    output_bag.close()