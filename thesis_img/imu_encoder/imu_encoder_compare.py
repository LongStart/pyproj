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

f_encoder = open('build/encoder.txt','w')
f_imu = open('build/imu.txt','w')
f_gyro_acc_adc = open('build/gyro_acc_adc.txt','w')

for topic, msg, t in bag.read_messages(topics=['/encoder_sensor_puber']):
    # if((0 == len(encoder_t)) or (t.to_sec() - encoder_t[-1] > 1e-3)):
    encoder_data += [msg.vector.x]
    encoder_t += [t.to_sec()]
    f_encoder.write("{:.8f} {}\n".format(t.to_sec(), msg.vector.x))

for topic, msg, t in bag.read_messages(topics=['/euler_puber']):
    imu_data += [msg.vector.y]
    imu_t += [t.to_sec()]
    f_imu.write("{} {}\n".format(t.to_sec(), msg.vector.y))

for topic, msg, t in bag.read_messages(topics=['/imu_puber']):
    f_gyro_acc_adc.write("{} {} {} {}\n".format(t.to_sec(), msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))

f_encoder.close()
f_imu.close()

plt.plot(encoder_t, encoder_data)
plt.plot(imu_t, imu_data)
plt.show()


