#!/usr/bin/env python

import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from sys import argv
import os

def tf_message_to_pose_stamped(tf_msg):
    pose_msg = PoseStamped()
    pose_msg.header             = tf_msg.transforms[0].header
    pose_msg.pose.position      = tf_msg.transforms[0].transform.translation
    pose_msg.pose.orientation   = tf_msg.transforms[0].transform.rotation
    return pose_msg


if __name__ == "__main__":
    if(len(argv) < 4):
        print('ex: python extract_tf_bag_to_pose_stamped.py tf.bag mother_frame child_frame')
        quit()
    input_bag_name = argv[1]
    mother_frame = argv[2]
    child_frame = argv[3]

    output_bag_name = os.path.splitext(input_bag_name)[0] + '_pose_stamped.bag'
    output_bag = rosbag.Bag(output_bag_name, 'w')
    input_bag = rosbag.Bag(input_bag_name)
    output_topic_name = '/' + child_frame + '_to_' + mother_frame + '_loc'

    for topic, msg, t in input_bag.read_messages(topics=['/tf']):
        if msg.transforms[0].header.frame_id != mother_frame or msg.transforms[0].child_frame_id != child_frame:
            continue
        pose_msg = tf_message_to_pose_stamped(msg)
        output_bag.write(output_topic_name, pose_msg, t)
    input_bag.close()
    output_bag.close()