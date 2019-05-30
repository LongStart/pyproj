#!/usr/bin/env python

import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from sys import argv
import os

# def nav_path_message_to_pose_stamped(nav_path_msg):
#     return nav_path_msg.poses


if __name__ == "__main__":
    if(len(argv) < 3):
        print('ex: python extract_tf_bag_to_pose_stamped.py nav_path.bag nav_topic_name')
        quit()
    input_bag_name = argv[1]
    nav_topic_name = argv[2]

    output_bag_name = os.path.splitext(input_bag_name)[0] + '_pose_stamped.bag'
    output_bag = rosbag.Bag(output_bag_name, 'w')
    input_bag = rosbag.Bag(input_bag_name)

    nav_msgs = [msg for topic, msg, t in input_bag.read_messages(topics=[nav_topic_name])]
    # nav_msgs = input_bag.read_messages(topics=[nav_topic_name])
    for pose in nav_msgs[-1].poses:
        output_bag.write(nav_topic_name, pose, pose.header.stamp)
    
    input_bag.close()
    output_bag.close()