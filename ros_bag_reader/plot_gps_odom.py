import matplotlib.pyplot as plt
import json
from math import *
from loc_file_to_list import *
from odom_file_to_list import *

odom_file_name = "build/0830_bag_odometry.json"
gps_file_name = "data/0830_bag_utm_gps_out.json"
loc_file_name = "../../cppproj/ros_test/bag_tf_extraction/build/output_pose.txt"

gps_x = []
gps_y = []

a = 0.67

gps_dict = json.load(open(gps_file_name))['sections'][0]

gps_x_off = gps_dict['positions'][0]['x']
gps_y_off = gps_dict['positions'][0]['y']

for position in gps_dict['positions']:
    x = position['x'] - gps_x_off
    y = position['y'] - gps_y_off
    gps_x += [x*cos(a) - y*sin(a)]
    gps_y += [x*sin(a) + y*cos(a)]

loc_x, loc_y = loc_file_to_list(loc_file_name, 0.67)
odom_x, odom_y = odom_file_to_list(odom_file_name)

plt.plot(odom_x, odom_y, '.')
plt.plot(gps_x, gps_y, '.-')
plt.plot(loc_x, loc_y, )
plt.axis('equal')
plt.show()

