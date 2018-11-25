from euler_to_quaternion import euler_to_quaternion
from geometry_msgs.msg import PoseStamped
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