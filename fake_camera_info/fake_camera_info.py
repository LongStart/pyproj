import rospy
import genpy
import yaml
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sys import argv
import os

def format_calibration_to_msg(calib_format):
    msg_format = {}
    msg_format["header"] = {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': ''}
    msg_format["width"] = calib_format["image_width"]
    msg_format["height"] = calib_format["image_height"]
    msg_format["distortion_model"] = 'plumb_bob'
    msg_format["D"] = calib_format['distortion_coefficients']['data']
    msg_format["K"] = calib_format['camera_matrix']['data']
    msg_format["R"] = calib_format['rectification_matrix']['data']
    msg_format["P"] = calib_format['projection_matrix']['data']
    msg_format["binning_x"] = 0
    msg_format["binning_y"] = 0
    msg_format["roi"] = {'x_offset': 0, 'y_offset': 0, 'height': 0, 'width': 0, 'do_rectify': False}
    return msg_format

def create_info_message(yaml_file):
    text = open(yaml_file,'r')
    info = yaml.safe_load(text)
    info_msg = CameraInfo()
    genpy.message.fill_message_args(info_msg, format_calibration_to_msg(info))
    return info_msg

def image_callback(msg, args):
    camera_info_pub = args[0]
    camera_info = args[1]
    camera_info.header.stamp = msg.header.stamp
    camera_info_pub.publish(camera_info)

def process(unstamped_camera_info, camera_info_topic_name, image_topic_name):
    rospy.init_node('camera_info_generator')
    camera_info_pub = rospy.Publisher(camera_info_topic_name, CameraInfo, queue_size=10)
    rospy.Subscriber(image_topic_name, Image, image_callback, (camera_info_pub, unstamped_camera_info))
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    # execute only if run as a script
    if(len(argv) < 2):
        print("ex: python p20_calibration.yaml /camera/image_raw")
        quit()
    yaml_file = argv[1]
    image_topic_name = argv[2]
    camera_info_topic_name = os.path.dirname(image_topic_name) + '/camera_info'
    unstamped_camera_info = create_info_message(yaml_file)
    process(unstamped_camera_info, camera_info_topic_name, image_topic_name)