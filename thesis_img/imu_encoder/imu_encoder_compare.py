#!/usr/bin/env python
# coding=utf-8

import rosbag
import sys
import matplotlib.pyplot as plt

if(len(sys.argv) < 2):
    print("Please input bag file name")
    print("Example: python plot_gps.py bag_path")

bag_path = sys.argv[1]
bag = rosbag.Bag(bag_path)
encoder_data = []
encoder_t = []

imu_data = []
imu_t = []

for topic, msg, t in bag.read_messages(topics=['/encoder_sensor_puber']):
    encoder_data += [msg.vector.x]
    print(msg.vector.x)
    encoder_t += [t.to_sec()]

for topic, msg, t in bag.read_messages(topics=['/euler_puber']):
    imu_data += [msg.vector.y]
    # print(msg.vector.x)
    imu_t += [t.to_sec()]

plt.plot(encoder_t, encoder_data)
plt.plot(imu_t, imu_data)
plt.show()


