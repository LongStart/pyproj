#!/usr/bin/env python

import rospy
import genpy
import yaml
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from sys import argv
import os

def tf_callback(msg, args):
    if msg.transforms[0].header.frame_id != 'map' or msg.transforms[0].child_frame_id != 'base_link':
        # print('not what I want')
        return
        
    loc_pub = args
    localization = PoseStamped()
    localization.header = msg.transforms[0].header
    localization.pose.position = msg.transforms[0].transform.translation
    localization.pose.orientation = msg.transforms[0].transform.rotation
    loc_pub.publish(localization)

def process(output_topic_name, input_topic_name):
    rospy.init_node('localization_republisher')
    loc_pub = rospy.Publisher(output_topic_name, PoseStamped, queue_size=20)
    rospy.Subscriber(input_topic_name, TFMessage, tf_callback, (loc_pub))
    rospy.spin()

if __name__ == "__main__":
    input_topic_name = '/tf'
    output_topic_name = '/localization'
    process(output_topic_name, input_topic_name)