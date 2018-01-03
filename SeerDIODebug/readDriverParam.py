import struct
import CanFrame_pb2
import socket
import json

F4KAddress = ('192.168.192.4', 15003)


def addsingleIDlistenmailbox():
    PACK_HEAD = 0x0000104C
    SUBCOMMAND = 1
    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.settimeout(2)
    frame = CanFrame_pb2.CanFrame()
    frame.ID = 0x05
    frame.Channel = 1
    frame.Extended = False
    frame.Remote = False
    serialized_pb_CAN_msg = frame.SerializeToString()
    rawmsg = struct.pack('<2I' + str(len(serialized_pb_CAN_msg)) +
                         's', PACK_HEAD, SUBCOMMAND, serialized_pb_CAN_msg)

    so.sendto(rawmsg, F4KAddress)
    try:
        (data, remoteaddr) = so.recvfrom(1024)
    except socket.timeout:
        so.close()
        return -1

    (head, subcmd, mailboxaddr) = struct.unpack('<3I', data)
    if(head != PACK_HEAD):
        print('pack head mismatch')
    if (subcmd != SUBCOMMAND):
        print('subcmd mismatch')

    so.close()
    return mailboxaddr


def querydrivervalue(valueindex, paramtype):
    PACK_HEAD = 0x00001017
    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.settimeout(2)
    frame = CanFrame_pb2.CanFrame()
    frame.ID = 0x06
    frame.Channel = 1
    frame.Extended = False
    frame.Remote = False
    frame.DLC = 6
    frame.Data = struct.pack('>I2b', 0, valueindex, 0x02)

    serialized_pb_CAN_msg = frame.SerializeToString()
    rawmsg = struct.pack('<I' + str(len(serialized_pb_CAN_msg)
                                    ) + 's', PACK_HEAD, serialized_pb_CAN_msg)

    so.sendto(rawmsg, F4KAddress)

    try:
        (data, remoteaddr) = so.recvfrom(1024)
    except socket.timeout:
        print('timeout111')
        so.close()
        return -1

    (msgId, pbdata) = struct.unpack('<I' + str(len(data) - 4) + 's', data)
    if(0x00001019 == msgId):
        rxframe = CanFrame_pb2.CanFrame()
        try:
            rxframe.ParseFromString(pbdata)
        except:
            print('pbdata parse error')
            print(data)
            os.system('pause')
            so.close()
            return -1

        # if(paramtype == 'I'):
        #     (value, b1, b2) = struct.unpack('>I2b', rxframe.Data)
        # elif(paramtype == 'f'):
        #     (value, b1, b2) = struct.unpack('<f2b', rxframe.Data)
        (value, b1, b2) = struct.unpack('>' + paramtype + '2b', rxframe.Data)
        so.close()
        return value


objectdict = {
    'CAN_ID_A': {'idx': 1, 'type': 'I', 'value': 0},
    'CAN_ID_A_back': {'idx': 2, 'type': 'I', 'value': 0},
    'CAN_ID_B': {'idx': 3, 'type': 'I', 'value': 0},
    'CAN_ID_B_back': {'idx': 4, 'type': 'I', 'value': 0},
    'CAN_ID_config': {'idx': 5, 'type': 'I', 'value': 0},
    'CAN_ID_config_back': {'idx': 6, 'type': 'I', 'value': 0},
    'CAN_bitrate': {'idx': 7, 'type': 'I', 'value': 0},
    'system_velocity_mode': {'idx': 16, 'type': 'I', 'value': 0},
    'motor_num_pole_pairs': {'idx': 17, 'type': 'I', 'value': 0},
    'motor_Rr': {'idx': 18, 'type': 'f', 'value': 0},
    'motor_Rs': {'idx': 19, 'type': 'f', 'value': 0},
    'motor_Ls_d': {'idx': 20, 'type': 'f', 'value': 0},
    'motor_Ls_q': {'idx': 21, 'type': 'f', 'value': 0},
    'motor_rated_flux': {'idx': 22, 'type': 'f', 'value': 0},
    'motor_max_current': {'idx': 23, 'type': 'f', 'value': 0},
    'motor_encoder_lines': {'idx': 24, 'type': 'f', 'value': 0},
    'motor_max_speed_krpm': {'idx': 25, 'type': 'f', 'value': 0},
    'motor_id_adjust_offset_pu': {'idx': 26, 'type': 'f', 'value': 0},
    'motor_max_offset_counts': {'idx': 27, 'type': 'I', 'value': 0},
    'motor_offset_rotation_counts': {'idx': 28, 'type': 'I', 'value': 0},
    'motor_dual_lock': {'idx': 29, 'type': 'I', 'value': 0},
}

addsingleIDlistenmailbox()
for key in objectdict:
    value = querydrivervalue(objectdict[key]['idx'], objectdict[key]['type'])
    objectdict[key]['value'] = value

driverconfig = json.dumps(objectdict, indent=4)
# print(driverconfig)
f = open('./driverconfig.json', 'w')
f.write(driverconfig)
f.close()
