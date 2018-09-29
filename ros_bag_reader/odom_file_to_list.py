import json
from math import *

# odom_file_name = "build/0830_bag_odometry.json"
# gps_file_name = "data/0830_bag_utm_gps_out.json"
# loc_file_name = "../../cppproj/ros_test/bag_tf_extraction/build/output_pose.txt"

def odom_file_to_list(odom_file_name):
    
    odom_x = []
    odom_y = []

    odom_dict = json.load(open(odom_file_name))['sections'][0]

    odom_x_off = odom_dict['poses'][0]['transform']['translation']['x']
    odom_y_off = odom_dict['poses'][0]['transform']['translation']['y']

    for pose in odom_dict['poses']:
        x = pose['transform']['translation']['x'] - odom_x_off
        y = pose['transform']['translation']['y'] - odom_y_off
        odom_x += [x]
        odom_y += [y]
    
    return (odom_x, odom_y)


