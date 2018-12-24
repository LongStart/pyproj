from geometry_msgs.msg import Vector3Stamped
import rospy
from math import *

def encode_encoder_sensor(encoder_raw_data_pack, line_num, offset):
    encoder_raw_data = encoder_raw_data_pack[0]
    sensor_angle = (-0.25 - float(encoder_raw_data + offset) / (line_num * 4)) * 360
    # sensor_angle += offset

    if(sensor_angle > 180):
         sensor_angle -= 360
    if(sensor_angle < -180):
         sensor_angle += 360

    angle_msg = Vector3Stamped()
    angle_msg.vector.x = sensor_angle

    angle_msg.header.frame_id = "local_frame"
    angle_msg.header.stamp = rospy.get_rostime()
    return angle_msg

def encode_encoder_sensor_rad(encoder_raw_data_pack, line_num, offset):
    encoder_raw_data = encoder_raw_data_pack[0]
    sensor_angle = (float(encoder_raw_data + offset) / (line_num * 4)) * 2*pi - 100

    angle_msg = Vector3Stamped()
    angle_msg.vector.x = sensor_angle

    angle_msg.header.frame_id = "local_frame"
    angle_msg.header.stamp = rospy.get_rostime()
    return angle_msg

def delta_angle(angle_1, angle_0):
    delta = angle_1 - angle_0
    if(delta > pi):
        delta -= 2*pi
    if(delta < -pi):
        delta += 2*pi

    return delta

def encode_encoder_angle_velocity(encoder_msg, prev_encoder_msg):
    dt = encoder_msg.header.stamp.to_sec() - prev_encoder_msg.header.stamp.to_sec()
    # dt = encoder_msg.header.stamp.to_nsec() - prev_encoder_msg.header.stamp.to_nsec()
    # if(dt < 1000000):
    #     print(dt)
    #     return None
    # print(dt)
    # dx = delta_angle(encoder_msg.vector.x, prev_encoder_msg.vector.x)#*1e9/dt
    # dy = delta_angle(encoder_msg.vector.y, prev_encoder_msg.vector.y)#*1e9/dt
    # dz = delta_angle(encoder_msg.vector.z, prev_encoder_msg.vector.z)#*1e9/dt

    dx = dt
    dy = dt
    dz = dt

    angle_vel_msg = Vector3Stamped()
    angle_vel_msg.header.frame_id = "local_frame"
    angle_vel_msg.header.stamp = encoder_msg.header.stamp
    angle_vel_msg.vector.x = dx
    angle_vel_msg.vector.y = dy
    angle_vel_msg.vector.z = dz

    return angle_vel_msg