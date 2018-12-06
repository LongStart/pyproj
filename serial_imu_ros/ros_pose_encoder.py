from euler_to_quaternion import euler_to_quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
import rospy

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