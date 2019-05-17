#!/usr/bin/env python
# coding=utf-8

import rosbag
import sys
import json
import glob
import os

if(len(sys.argv) < 2):
    print("Please input bag file name")
    print("Example: python odometry_to_json.py bagfile.bag")
# bags = glob.glob(glob_input)
bags = [sys.argv[1]]
output_dict = {"sections":[]}
print(bags)

for bag_path in bags:
    pos_x = []
    pos_y = []
    bag = rosbag.Bag(bag_path)
    section = {"poses":[]}
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        pose = {"t.secs":0, "t.nsecs":0, "transform":{"translation":{"x":0, "y":0, "z":0}, "rotation": {"x":0, "y":0, "z":0, "w":1}}}
        pose["transform"]["translation"]["x"] = msg.transforms[0].transform.translation.x
        pose["transform"]["translation"]["y"] = msg.transforms[0].transform.translation.y
        pose["transform"]["translation"]["z"] = msg.transforms[0].transform.translation.z

        pose["transform"]["rotation"]["x"] = msg.transforms[0].transform.rotation.x
        pose["transform"]["rotation"]["y"] = msg.transforms[0].transform.rotation.y
        pose["transform"]["rotation"]["z"] = msg.transforms[0].transform.rotation.z
        pose["transform"]["rotation"]["w"] = msg.transforms[0].transform.rotation.w

        pose['t.secs'] = msg.transforms[0].header.stamp.secs
        pose['t.nsecs'] = msg.transforms[0].header.stamp.nsecs

        section["poses"] += [pose]

    bag.close()
    output_dict["sections"] += [section]
    print(".")

outstring = json.dumps(output_dict,indent=4, ensure_ascii=False)
fileoutname = 'build/odometry.json'
fileout = open(fileoutname, 'w')
fileout.write(outstring)
fileout.close()
