import matplotlib.pyplot as plt
import json
from math import *
from loc_file_to_list import *

odom_file_name = "build/0830_bag_odometry.json"
gps_file_name = "data/0830_bag_utm_gps_out.json"
loc_file_name = "../../cppproj/ros_test/bag_tf_extraction/build/output_pose.txt"

odom_x = []
odom_y = []

gps_x = []
gps_y = []

a = 0.67

odom_dict = json.load(open(odom_file_name))['sections'][0]
gps_dict = json.load(open(gps_file_name))['sections'][0]

odom_x_off = odom_dict['poses'][0]['transform']['translation']['x']
odom_y_off = odom_dict['poses'][0]['transform']['translation']['y']

gps_x_off = gps_dict['positions'][0]['x']
gps_y_off = gps_dict['positions'][0]['y']

for pose in odom_dict['poses']:
    x = pose['transform']['translation']['x'] - odom_x_off
    y = pose['transform']['translation']['y'] - odom_y_off
    odom_x += [x]
    odom_y += [y]

for position in gps_dict['positions']:
    x = position['x'] - gps_x_off
    y = position['y'] - gps_y_off
    gps_x += [x*cos(a) - y*sin(a)]
    gps_y += [x*sin(a) + y*cos(a)]

loc_x, loc_y = loc_file_to_list(loc_file_name, 0.67)

plt.plot(odom_x, odom_y)
plt.plot(gps_x, gps_y, '.-')
plt.plot(loc_x, loc_y, )
plt.axis('equal')
plt.show()

