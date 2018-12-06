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
from calibration_server import *
from ros_pose_encoder import *
from calibration_service_test.srv import *

if(len(sys.argv) < 2):
    print("No input COM port, example: python.exe intboardpowermonitor.py COM5")
    input("Press enter to exit...")
    quit()
com_name = sys.argv[1]

com = serial.Serial(com_name, baudrate=115200, timeout=1)

def handle_calibration_l(req):
    calibration_commd = b'\xa5\x5a\x04\xe0\xe4\xaa'
    com.write(calibration_commd)
    return CalibrationResponse(56)

pub_euler = rospy.Publisher('euler_puber', Vector3, queue_size=10)
pub_pose = rospy.Publisher('pose_puber', PoseStamped, queue_size=10)
server_calib = rospy.Service('calibrate_imu', Calibration, handle_calibration_l)
rospy.init_node('imu_msg_converter', anonymous=True)

buffer = bytearray(b'')

while(True):
    cmd = b''
    buff_len = len(buffer)
    if(buff_len > 5 and 0xa5 == buffer[0] and 0x5a == buffer[1] and buff_len >= (buffer[2] + 2) ):
        pack = decode_from_buffer(buffer[2:2+buffer[2]])
        buffer = buffer[buffer[2] + 2:]
        if(3 == len(pack)):
            pub_pose.publish(encode_ros_pose(pack))
            pub_euler.publish(encode_ros_euler(pack))
            

    try:
        cmd = com.read(1)
    except KeyboardInterrupt:
        print('Monitor closed')
        break

    if(cmd == b''):
        print('No message received!')
        buffer = bytearray(b'')
        continue

    buffer += cmd




# file_out.close()
rospy.spin()
com.close()
