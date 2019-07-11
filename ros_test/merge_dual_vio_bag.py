#!/usr/bin/env python

import rospy
import rosbag
import os
from sys import argv


if __name__ == "__main__":
    if(len(argv) < 3):
        print('ex: python downsample_bag.py input1.bag input2.bag')
        quit()
    
    bag_num = len(argv) - 1
    in_bagnames = []
    name_prefixs = []
    output_bagname = ''
    for i in range(bag_num): 
        in_bagnames.append(argv[1 + i])
        name_prefixs.append(os.path.basename(in_bagnames[i]).split('_')[0])
        output_bagname += (name_prefixs[i] + '_')
    output_bagname += 'merged.bag'
    
    output_bag = rosbag.Bag(output_bagname, 'w')
    start_time = 0.
    with rosbag.Bag(in_bagnames[0]) as first_bag:
        start_time = first_bag.get_start_time()
    
    for i in range(bag_num):
        input_bag = rosbag.Bag(in_bagnames[i])
        dt = rospy.rostime.Time(input_bag.get_start_time() - start_time)
        count = 0
        for topic, msg, t in input_bag.read_messages():
            msg.header.stamp = msg.header.stamp - dt
            output_bag.write('/' + name_prefixs[i] + topic, msg, t - dt)
            count += 1
            if count % 2000 == 0:
                print('bag[{}]: {}/{}msgs'.format(i, count, input_bag.get_message_count()))
        input_bag.close()
    output_bag.close()