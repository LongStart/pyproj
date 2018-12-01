import struct

def decode_raw_adc_data(data_field):
    raw_adc_format_string = '>9h'
    mid_data = struct.unpack(raw_adc_format_string, data_field)
    return mid_data

def strange_convert(raw_value):
    temp = 0
    if(raw_value & 0x8000):
        temp = 0 - (raw_value & 0x7fff)
    else:
        temp = (raw_value & 0x7fff)
    return temp / 10.
    
def decode_euler_angle(data_field):
    euler_angle_format_string = '>7h'
    mid_data = struct.unpack(euler_angle_format_string, data_field)

    yaw = strange_convert(mid_data[0])
    pitch = strange_convert(mid_data[1])
    roll = strange_convert(mid_data[2])
    return (yaw, pitch, roll)

def imu_check_sum(target_msg):
    return sum(target_msg) & 0xff

def decode_from_buffer(raw_msg):
    adc_frame_type_id = 0xa2
    rpy_frame_type_id = 0xa1
    frame_end_byte = 0xaa

    if(len(raw_msg) != raw_msg[0]):
        print("length mismatch, actual length: {0}, get: {1}".format(len(raw_msg), raw_msg[0]))
        return None

    if(raw_msg[-1] != frame_end_byte):
        print("pack end: {0}, should be {1}".format(raw_msg[-1], frame_end_byte))
        return None

    if(imu_check_sum(raw_msg[:-2]) != raw_msg[-2]):
        print("sum check faild, should be: 0x{:x}, get: 0x{:x}".format(imu_check_sum(raw_msg[:-2]), raw_msg[-2]))
        return None

    if(raw_msg[1] == adc_frame_type_id):
        return decode_raw_adc_data(raw_msg[2:-2])
    elif(raw_msg[1] == rpy_frame_type_id):
        # print([v for v in raw_msg])
        return decode_euler_angle(raw_msg[2:-2])
    else:
        print('unknow type')
        return None

    