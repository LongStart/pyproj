import rospy
import serial
from serial import *
import sys
import time
import os
import threading
import tf
from decode_bytearray import *
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from euler_to_quaternion import euler_to_quaternion
from math import pi

if(len(sys.argv) < 2):
    print("No input COM port, example: python.exe intboardpowermonitor.py COM5")
    input("Press enter to exit...")
    quit()
com_name = sys.argv[1]

com = serial.Serial(com_name, baudrate=115200, timeout=1)
pub_euler = rospy.Publisher('euler_puber', Vector3, queue_size=10)
pub_pose = rospy.Publisher('pose_puber', PoseStamped, queue_size=10)
rospy.init_node('imu_msg_converter', anonymous=True)

buffer = bytearray(b'')

while(True):
    cmd = b''
    buff_len = len(buffer)
    if(buff_len > 5 and 0xa5 == buffer[0] and 0x5a == buffer[1] and buff_len >= (buffer[2] + 2) ):
        pack = decode_from_buffer(buffer[2:2+buffer[2]])
        buffer = buffer[buffer[2] + 2:]
        if(3 == len(pack)):
            # euler_angle = Vector3()
            yaw   = pack[0] 
            pitch = pack[1]
            roll  = pack[2]

            # euler_angle.x
            # euler_angle.y
            # euler_angle.z
            # pub_euler.publish(euler_angle)

            pose = PoseStamped()
            
            (qx,qy,qz,qw) = euler_to_quaternion(yaw, pitch, roll)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            # pose.pose.orientation = Quaternion(
            #     *tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'szyx'))
            pose.header.frame_id = "local_frame"
            pose.header.stamp = rospy.get_rostime()
            pub_pose.publish(pose)
            # print(pack)

    try:
        cmd = com.read(1)
    except KeyboardInterrupt:
        print('Monitor closed')
        break

    if(cmd == b''):
        print('No message received!')
        buffer.clear()
        continue

    buffer += cmd




# file_out.close()
com.close()
