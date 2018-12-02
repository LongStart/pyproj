from geometry_msgs.msg import Vector3
import rospy

def encode_encoder_sensor(encoder_raw_data, line_num, offset):
    sensor_angle = encoder_raw_data / (line_num * 4) - offset 

    angle_msg = Vector3()
    angle_msg.x = sensor_angle

    pose.header.frame_id = "local_frame"
    pose.header.stamp = rospy.get_rostime()
    return angle_msg