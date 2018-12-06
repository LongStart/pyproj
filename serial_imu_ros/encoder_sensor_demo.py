import rospy
import serial
from serial import *
import sys
import time
import os
import threading
import tf
from decode_bytearray import *
from geometry_msgs.msg import Vector3Stamped
from ros_sensor_encoder import *

if(len(sys.argv) < 2):
    print("No input COM port, example: python3 encoder_sensor_demo.py COM5")
    input("Press enter to exit...")
    quit()
com_name = sys.argv[1]

com = serial.Serial(com_name, baudrate=115200, timeout=.05)

pub_encoder = rospy.Publisher('encoder_sensor_puber', Vector3Stamped, queue_size=10)
rospy.init_node('encoder_sensor_msg_converter', anonymous=True)

buffer = bytearray(b'')
prev_pack = ()

while(True):
    cmd = b''
    buff_len = len(buffer)
    if(buff_len > 5 and 0xa5 == buffer[0] and 0x5a == buffer[1] and buff_len >= (buffer[2] + 2) ):
        pack = decode_from_buffer(buffer[2:2+buffer[2]])
        buffer = buffer[buffer[2] + 2:]
        if(1 == len(pack)):
            # print(pack[0])
            pub_encoder.publish(encode_encoder_sensor(pack, 500, 0))
            prev_pack = pack

    try:
        cmd = com.read(1)
    except KeyboardInterrupt:
        print('Monitor closed')
        break

    if(cmd == b''):
        
        buffer = bytearray(b'')
        if(len(prev_pack) > 0):
            pub_encoder.publish(encode_encoder_sensor(prev_pack, 500, 0))
        continue

    buffer += cmd




# file_out.close()
rospy.spin()
com.close()
