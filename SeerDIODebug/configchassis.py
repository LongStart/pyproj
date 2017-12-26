import re
import json
import message_chassisconfig_pb2
import struct
import socket
import sys


def configf4kernelchassis(model_file):

    with open(model_file, 'r', encoding='utf-8') as f:
        f_uncomment = re.sub(r'\/\/[^\n]*', '', f.read())
        data = json.loads(f_uncomment)
        f.close()

    frame = message_chassisconfig_pb2.Message_ChassisConfig()
    frame.E = data['chassis']['wheelbase']
    frame.encoderLines = data['chassis']['encoderLine']
    frame.reductionRatio = data['chassis']['reductionRatio']
    frame.R[:] = [data['chassis']['wheelRadius'],
                  data['chassis']['wheelRadius']]

    print(data['chassis'])

    CHASSIS_CONFIG_HEAD = 0x00000002
    SeerDIO_addr = ('192.168.192.4', 15003)
    F4kAddr_old = ('192.168.192.4', 5003)

    serialized_pb_config_msg = frame.SerializeToString()
    seerDioCmder_msg = struct.pack('<I' + str(len(serialized_pb_config_msg)) +
                                   's', CHASSIS_CONFIG_HEAD, serialized_pb_config_msg)
    heartbeatMsg = struct.pack('<2I', 0x200, 0xffffffff)

    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.settimeout(2)
    so.sendto(seerDioCmder_msg, SeerDIO_addr)
    so.sendto(heartbeatMsg, F4kAddr_old)

    config_drivertype_msg = struct.pack('<2I', 1, data['chassis']['driver'])
    so.sendto(config_drivertype_msg, SeerDIO_addr)

    so.close()
