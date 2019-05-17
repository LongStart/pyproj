#!/usr/bin/env python
# coding=utf-8

import sys
import json
import matplotlib.pyplot as plt


if(len(sys.argv) < 2):
    print("Please input bag file name")
    print("Example: python plot_json_gps.py bag_path")

input_file_name = sys.argv[1]
position_dict = json.load(open(input_file_name))



for section in position_dict["sections"]:
    x = []
    y = []
    for position in section["positions"]:
        # x += [position["latitude"]]
        # y += [position["longitude"]]
        y += [position["latitude"]]
        x += [position["longitude"]]

    plt.plot(x,y,'o-')
    # plt.pause(0.2)

plt.axis('equal')
plt.show()
