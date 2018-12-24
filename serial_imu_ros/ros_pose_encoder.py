from euler_to_quaternion import euler_to_quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
import rospy
from sensor_msgs.msg import Imu
from math import *

def encode_ros_pose(ypr_1_10_degree):
    yaw   = ypr_1_10_degree[0] 
    pitch = ypr_1_10_degree[1]
    roll  = ypr_1_10_degree[2]

    pose = PoseStamped()
    
    (qx,qy,qz,qw) = euler_to_quaternion(yaw, pitch, roll)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    pose.header.frame_id = "local_frame"
    pose.header.stamp = rospy.get_rostime()
    return pose

def encode_ros_euler(ypr_1_10_degree):
    yaw   = ypr_1_10_degree[0] 
    pitch = ypr_1_10_degree[1]
    roll  = ypr_1_10_degree[2]

    euler_vec = Vector3Stamped()
    euler_vec.vector.x = yaw
    euler_vec.vector.y = pitch
    euler_vec.vector.z = roll

    euler_vec.header.frame_id = "local_frame"
    euler_vec.header.stamp = rospy.get_rostime()
    
    return euler_vec

def encode_ros_euler_rad(ypr_1_10_degree):
    yaw   = ypr_1_10_degree[0] 
    pitch = ypr_1_10_degree[1]
    roll  = ypr_1_10_degree[2]

    euler_vec = Vector3Stamped()
    euler_vec.vector.x = yaw / 180 * pi
    euler_vec.vector.y = pitch / 180 * pi
    euler_vec.vector.z = roll / 180 * pi

    euler_vec.header.frame_id = "local_frame"
    euler_vec.header.stamp = rospy.get_rostime()
    
    return euler_vec

def encode_imu_msg(raw_sensor_int16):
    ax = float(raw_sensor_int16[0])
    ay = float(raw_sensor_int16[1])
    az = float(raw_sensor_int16[2])
    gx = float(raw_sensor_int16[3])/1000.
    gy = float(raw_sensor_int16[4])/1000.
    gz = float(raw_sensor_int16[5])/1000.

    imu_msg = Imu()
    imu_msg.header.frame_id = "local_frame"
    imu_msg.header.stamp = rospy.get_rostime()

    imu_msg.linear_acceleration.x = ax
    imu_msg.linear_acceleration.y = ay
    imu_msg.linear_acceleration.z = az

    imu_msg.angular_velocity.x = gx
    imu_msg.angular_velocity.y = gy
    imu_msg.angular_velocity.z = gz

    return imu_msg

    
