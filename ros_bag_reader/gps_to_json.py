#!/usr/bin/env python
# coding=utf-8

import rosbag
import sys
import json
import glob
import os

if(len(sys.argv) < 2):
    print("Please input bag file name")
    print("Example: python plot_gps.py bag_path")
glob_input = sys.argv[1] + "/*.bag"
bags = glob.glob(glob_input)
output_dict = {"sections":[]}
print(bags)

for bag_path in bags:
    gps_poses = []
    pos_x = []
    pos_y = []
    bag = rosbag.Bag(bag_path)
    section = {"positions":[]}
    for topic, msg, t in bag.read_messages(topics=['/vehicle/seriesGPSraw']):
        gps_poses += [msg]
        section["positions"] += [{"latitude":msg.latitude, "longitude": msg.longitude}]
    bag.close()
    output_dict["sections"] += [section]
    print(".")

outstring = json.dumps(output_dict,indent=4, ensure_ascii=False)
fileoutname = 'build/test1.json'
fileout = open(fileoutname, 'w')
fileout.write(outstring)
fileout.close()
