from geometry_msgs.msg import Vector3Stamped
import rospy

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