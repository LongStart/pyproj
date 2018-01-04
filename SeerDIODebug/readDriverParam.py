import struct
import CanFrame_pb2
import socket
import json
import time
import sys
import traceback

# grep ^(..)\n(.*):('.')\n(.*)\n(.*)
# grep '$2':{'idx': 0x$1, 'type': $3, 'ref': '$4', 'ret': '$5'},

objectdict = {
    'CAN_ID_A': {'idx': 0x01, 'type': 'I', 'ref': '（只串口可配）Driver A CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_ID_A_back': {'idx': 0x02, 'type': 'I', 'ref': '（只串口可配）Controller A CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_ID_B': {'idx': 0x03, 'type': 'I', 'ref': '（只串口可配）Driver B CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_ID_B_back': {'idx': 0x04, 'type': 'I', 'ref': '（只串口可配）Controller B CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_ID_config': {'idx': 0x05, 'type': 'I', 'ref': '（只串口可配）Configuration CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_ID_config_back': {'idx': 0x06, 'type': 'I', 'ref': '（只串口可配）Config back CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_bitrate': {'idx': 0x07, 'type': 'I', 'ref': '（只串口可配）CAN比特率。: 00:1M，01:500K，02:250K，03:125K，04:100K', 'ret': '0'},
    'system_velocity_mode': {'idx': 0x10, 'type': 'I', 'ref': '（重新初始化生效）控制模式:0:spintac，1:pid', 'ret': '0:正常，1:错误数值'},
    'motor_num_pole_pairs': {'idx': 0x11, 'type': 'I', 'ref': '（重新初始化生效 ）电机极对数', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_Rr': {'idx': 0x12, 'type': 'f', 'ref': '（重新初始化生效 ）转子电阻（默认0，永磁同步机无绕组）', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_Rs': {'idx': 0x13, 'type': 'f', 'ref': '（重新初始化生效 ）定子绕组', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_Ls_d': {'idx': 0x14, 'type': 'f', 'ref': '（重新初始化生效 ）定子d轴电感', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_Ls_q': {'idx': 0x15, 'type': 'f', 'ref': '（重新初始化生效 ）定子q轴电感', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_rated_flux': {'idx': 0x16, 'type': 'f', 'ref': '（重新初始化生效 ）电机额定磁通', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_max_current': {'idx': 0x17, 'type': 'f', 'ref': '（重新初始化生效 ）电机最大电流', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_encoder_lines': {'idx': 0x18, 'type': 'f', 'ref': '（重新初始化生效 ）电机码盘线数', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_max_speed_krpm': {'idx': 0x19, 'type': 'f', 'ref': '（重新初始化生效 ）电机最大转速（千转）', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_id_adjust_offset_pu': {'idx': 0x1A, 'type': 'f', 'ref': '（重新初始化生效 ）系统旋转标定时的d轴直流标量', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_max_offset_counts': {'idx': 0x1B, 'type': 'I', 'ref': '（重新初始化生效 ）系统旋转检测超时时间', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_offset_rotation_counts': {'idx': 0x1C, 'type': 'I', 'ref': '（重新初始化生效 ）直流定位结束，启动旋转检测时的时间', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_dual_lock': {'idx': 0x1D, 'type': 'I', 'ref': '（重新初始化生效）是否两机联动状态。默认应该为true :1 true, 0 false', 'ret': '0:正常，1:错误数值'},
    'motor_max_acceleration': {'idx': 0x30, 'type': 'I', 'ref': '最大加速度，单位:1000 counts/second^2', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Id_Kp_multiplied_factor ': {'idx': 0x31, 'type': 'f', 'ref': 'Id_Kp的乘因子，默认0.25', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Id_Ki_multiplied_factor': {'idx': 0x32, 'type': 'f', 'ref': 'Id_Ki的乘因子，默认1.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Id_Kd': {'idx': 0x33, 'type': 'f', 'ref': '默认0.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Id_max': {'idx': 0x34, 'type': 'f', 'ref': '默认0.95', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Id_min': {'idx': 0x35, 'type': 'f', 'ref': '默认-0.95', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Iq_Kp_multiplied_factor ': {'idx': 0x36, 'type': 'f', 'ref': 'Iq_Kp的乘因子，默认0.25', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Iq_Ki_multiplied _factor': {'idx': 0x37, 'type': 'f', 'ref': 'Iq_Ki的乘因子，默认1.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Iq_Kd': {'idx': 0x38, 'type': 'f', 'ref': '默认0.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Iq_max': {'idx': 0x39, 'type': 'f', 'ref': '默认0.95', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Iq_min': {'idx': 0x3A, 'type': 'f', 'ref': '默认-0.95', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'spd_kp_multiplied_factor': {'idx': 0x3B, 'type': 'f', 'ref': 'spd_kp的乘因子，默认0.02', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'spd_ki_multiplied_factor': {'idx': 0x3C, 'type': 'f', 'ref': 'spd_ki的乘因子，默认2.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'spd_Kd': {'idx': 0x3D, 'type': 'f', 'ref': '默认0.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'spd_max': {'idx': 0x3E, 'type': 'f', 'ref': '默认1.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'spd_min': {'idx': 0x3F, 'type': 'f', 'ref': '默认-1.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'system_bandwidth': {'idx': 0x40, 'type': 'f', 'ref': '（重新初始化生效 ）spintac 系统带宽', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'system_inertia': {'idx': 0x41, 'type': 'f', 'ref': '（重新初始化生效 ）系统转动惯量', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'system_friction': {'idx': 0x42, 'type': 'f', 'ref': '（重新初始化生效 ）系统摩擦系数', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'is_motor_A_hall_enabled': {'idx': 0x50, 'type': 'I', 'ref': '是否启用电机A的霍尔纠正 1:启用，0:未启用', 'ret': '0:正常，1:错误数值'},
    'motor_A_hall_51': {'idx': 0x51, 'type': 'f', 'ref': 'hall_51，hall_15的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_A_hall_13': {'idx': 0x52, 'type': 'f', 'ref': 'hall_13，hall_31的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_A_hall_32': {'idx': 0x53, 'type': 'f', 'ref': 'hall_32，hall_23的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_A_hall_26': {'idx': 0x54, 'type': 'f', 'ref': 'hall_26，hall_62的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_A_hall_64': {'idx': 0x55, 'type': 'f', 'ref': 'hall_64，hall_46的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_A_hall_45': {'idx': 0x56, 'type': 'f', 'ref': 'hall_45，hall_54的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'is_motor_B_hall_enabled': {'idx': 0x60, 'type': 'I', 'ref': '是否启用电机B的霍尔矫正:1:启用，0:未启用', 'ret': '0:正常，1:错误数值'},
    'motor_B_hall_51': {'idx': 0x61, 'type': 'f', 'ref': 'hall_51，hall_15的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_B_hall_13': {'idx': 0x62, 'type': 'f', 'ref': 'hall_13，hall_31的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_B_hall_32': {'idx': 0x63, 'type': 'f', 'ref': 'hall_32，hall_23的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_B_hall_26': {'idx': 0x64, 'type': 'f', 'ref': 'hall_26，hall_62的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_B_hall_64': {'idx': 0x65, 'type': 'f', 'ref': 'hall_64，hall_46的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_B_hall_45': {'idx': 0x66, 'type': 'f', 'ref': 'hall_45，hall_54的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'}, 'CAN_ID_A': {'idx': 0x01, 'type': 'I', 'ref': '（只串口可配）Driver A CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_ID_A_back': {'idx': 0x02, 'type': 'I', 'ref': '（只串口可配）Controller A CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_ID_B': {'idx': 0x03, 'type': 'I', 'ref': '（只串口可配）Driver B CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_ID_B_back': {'idx': 0x04, 'type': 'I', 'ref': '（只串口可配）Controller B CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_ID_config': {'idx': 0x05, 'type': 'I', 'ref': '（只串口可配）Configuration CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_ID_config_back': {'idx': 0x06, 'type': 'I', 'ref': '（只串口可配）Config back CAN ID，支持标准帧，扩展帧', 'ret': '0'},
    'CAN_bitrate': {'idx': 0x07, 'type': 'I', 'ref': '（只串口可配）CAN比特率。: 00:1M，01:500K，02:250K，03:125K，04:100K', 'ret': '0'},
    'system_velocity_mode': {'idx': 0x10, 'type': 'I', 'ref': '（重新初始化生效）控制模式:0:spintac，1:pid', 'ret': '0:正常，1:错误数值'},
    'motor_num_pole_pairs': {'idx': 0x11, 'type': 'I', 'ref': '（重新初始化生效 ）电机极对数', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_Rr': {'idx': 0x12, 'type': 'f', 'ref': '（重新初始化生效 ）转子电阻（默认0，永磁同步机无绕组）', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_Rs': {'idx': 0x13, 'type': 'f', 'ref': '（重新初始化生效 ）定子绕组', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_Ls_d': {'idx': 0x14, 'type': 'f', 'ref': '（重新初始化生效 ）定子d轴电感', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_Ls_q': {'idx': 0x15, 'type': 'f', 'ref': '（重新初始化生效 ）定子q轴电感', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_rated_flux': {'idx': 0x16, 'type': 'f', 'ref': '（重新初始化生效 ）电机额定磁通', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_max_current': {'idx': 0x17, 'type': 'f', 'ref': '（重新初始化生效 ）电机最大电流', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_encoder_lines': {'idx': 0x18, 'type': 'f', 'ref': '（重新初始化生效 ）电机码盘线数', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_max_speed_krpm': {'idx': 0x19, 'type': 'f', 'ref': '（重新初始化生效 ）电机最大转速（千转）', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_id_adjust_offset_pu': {'idx': 0x1A, 'type': 'f', 'ref': '（重新初始化生效 ）系统旋转标定时的d轴直流标量', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_max_offset_counts': {'idx': 0x1B, 'type': 'I', 'ref': '（重新初始化生效 ）系统旋转检测超时时间', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_offset_rotation_counts': {'idx': 0x1C, 'type': 'I', 'ref': '（重新初始化生效 ）直流定位结束，启动旋转检测时的时间', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_dual_lock': {'idx': 0x1D, 'type': 'I', 'ref': '（重新初始化生效）是否两机联动状态。默认应该为true :1 true, 0 false', 'ret': '0:正常，1:错误数值'},
    'motor_max_acceleration': {'idx': 0x30, 'type': 'I', 'ref': '最大加速度，单位:1000 counts/second^2', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Id_Kp_multiplied_factor ': {'idx': 0x31, 'type': 'f', 'ref': 'Id_Kp的乘因子，默认0.25', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Id_Ki_multiplied_factor': {'idx': 0x32, 'type': 'f', 'ref': 'Id_Ki的乘因子，默认1.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Id_Kd': {'idx': 0x33, 'type': 'f', 'ref': '默认0.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Id_max': {'idx': 0x34, 'type': 'f', 'ref': '默认0.95', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Id_min': {'idx': 0x35, 'type': 'f', 'ref': '默认-0.95', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Iq_Kp_multiplied_factor ': {'idx': 0x36, 'type': 'f', 'ref': 'Iq_Kp的乘因子，默认0.25', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Iq_Ki_multiplied _factor': {'idx': 0x37, 'type': 'f', 'ref': 'Iq_Ki的乘因子，默认1.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Iq_Kd': {'idx': 0x38, 'type': 'f', 'ref': '默认0.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Iq_max': {'idx': 0x39, 'type': 'f', 'ref': '默认0.95', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'Iq_min': {'idx': 0x3A, 'type': 'f', 'ref': '默认-0.95', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'spd_kp_multiplied_factor': {'idx': 0x3B, 'type': 'f', 'ref': 'spd_kp的乘因子，默认0.02', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'spd_ki_multiplied_factor': {'idx': 0x3C, 'type': 'f', 'ref': 'spd_ki的乘因子，默认2.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'spd_Kd': {'idx': 0x3D, 'type': 'f', 'ref': '默认0.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'spd_max': {'idx': 0x3E, 'type': 'f', 'ref': '默认1.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'spd_min': {'idx': 0x3F, 'type': 'f', 'ref': '默认-1.0', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'system_bandwidth': {'idx': 0x40, 'type': 'f', 'ref': '（重新初始化生效 ）spintac 系统带宽', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'system_inertia': {'idx': 0x41, 'type': 'f', 'ref': '（重新初始化生效 ）系统转动惯量', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'system_friction': {'idx': 0x42, 'type': 'f', 'ref': '（重新初始化生效 ）系统摩擦系数', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'is_motor_A_hall_enabled': {'idx': 0x50, 'type': 'I', 'ref': '是否启用电机A的霍尔纠正 1:启用，0:未启用', 'ret': '0:正常，1:错误数值'},
    'motor_A_hall_51': {'idx': 0x51, 'type': 'f', 'ref': 'hall_51，hall_15的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_A_hall_13': {'idx': 0x52, 'type': 'f', 'ref': 'hall_13，hall_31的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_A_hall_32': {'idx': 0x53, 'type': 'f', 'ref': 'hall_32，hall_23的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_A_hall_26': {'idx': 0x54, 'type': 'f', 'ref': 'hall_26，hall_62的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_A_hall_64': {'idx': 0x55, 'type': 'f', 'ref': 'hall_64，hall_46的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_A_hall_45': {'idx': 0x56, 'type': 'f', 'ref': 'hall_45，hall_54的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'is_motor_B_hall_enabled': {'idx': 0x60, 'type': 'I', 'ref': '是否启用电机B的霍尔矫正:1:启用，0:未启用', 'ret': '0:正常，1:错误数值'},
    'motor_B_hall_51': {'idx': 0x61, 'type': 'f', 'ref': 'hall_51，hall_15的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_B_hall_13': {'idx': 0x62, 'type': 'f', 'ref': 'hall_13，hall_31的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_B_hall_32': {'idx': 0x63, 'type': 'f', 'ref': 'hall_32，hall_23的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_B_hall_26': {'idx': 0x64, 'type': 'f', 'ref': 'hall_26，hall_62的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_B_hall_64': {'idx': 0x65, 'type': 'f', 'ref': 'hall_64，hall_46的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
    'motor_B_hall_45': {'idx': 0x66, 'type': 'f', 'ref': 'hall_45，hall_54的纠正值', 'ret': '0:正常，1:设定值超出上限，2:设定值超出下限。'},
}


class CRLDriverConfiger(object):
    _F4KAddress = ('192.168.192.4', 15003)

    def __init__(self):
        self._so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._so.settimeout(0.01)
        print('Init driver configer, version 1.0.0')
        try:
            self._so.sendto(b'', ('', 123456))
        except:
            pass

    def confirmgatewayversion(self):
        PACK_HEAD = 0x00001032
        SUBCOMMAND = 0
        self._so.sendto(struct.pack('<2I', PACK_HEAD,
                                    SUBCOMMAND), self._F4KAddress)
        try:
            (data, remoteaddr) = self._so.recvfrom(1024)
        except socket.timeout:
            print('Cannot get version...')
            return False

        (head, v0, v1, v2) = struct.unpack('<4I', data)
        if(head != PACK_HEAD):
            print('Get gateway version pack head error: ' + str(head))
            return False
        if(v0 < 1 or v1 < 7 or v2 < 905):
            print(
                'Gateway version require at least 1.7.905, current: %d.%d.%d' % (v0, v1, v2))
            return False
        print('Gateway version confirm passed')
        return True

    def addsingleIDlistenmailbox(self):
        PACK_HEAD = 0x0000104C
        SUBCOMMAND = 1
        frame = CanFrame_pb2.CanFrame()
        frame.ID = 0x05
        frame.Channel = 1
        frame.Extended = False
        frame.Remote = False
        serialized_pb_CAN_msg = frame.SerializeToString()
        rawmsg = struct.pack('<2I' + str(len(serialized_pb_CAN_msg)) +
                             's', PACK_HEAD, SUBCOMMAND, serialized_pb_CAN_msg)

        self._so.sendto(rawmsg, self._F4KAddress)
        try:
            (data, remoteaddr) = self._so.recvfrom(1024)
        except socket.timeout:
            return -1

        (head, subcmd, mailboxaddr) = struct.unpack('<3I', data)
        if(head != PACK_HEAD):
            print('pack head mismatch')
        if (subcmd != SUBCOMMAND):
            print('subcmd mismatch')

        return mailboxaddr

    def deletelistenmailbox(self, mailboxaddr):
        PACK_HEAD = 0x0000104C
        SUBCOMMAND = 2
        param = mailboxaddr
        rawmsg = struct.pack('<3I', PACK_HEAD, SUBCOMMAND, param)
        self._so.sendto(rawmsg, self._F4KAddress)

        try:
            (data, remoteaddr) = self._so.recvfrom(1024)
        except socket.timeout:
            return -1

        (head, subcmd, ret) = struct.unpack('<3I', data)
        if(head != PACK_HEAD):
            print('pack head mismatch')
        if (subcmd != SUBCOMMAND):
            print('subcmd mismatch')

        if ret != 0:
            print('ret = ' + str(ret))
        return ret

    def senddrivercmd(self, cmdtype, valueindex, paramtype, value=0):
        PACK_HEAD = 0x00001017
        frame = CanFrame_pb2.CanFrame()
        frame.ID = 0x06
        frame.Channel = 1
        frame.Extended = False
        frame.Remote = False
        frame.DLC = 6
        rawcmd = 0x02
        if(cmdtype == 'w'):
            rawcmd = 0x01
        elif cmdtype == 'r':
            rawcmd = 0x02
        elif cmdtype == 's':
            rawcmd = 0x00

        frame.Data = struct.pack(
            '>' + paramtype + '2b', value, valueindex, rawcmd)

        serialized_pb_CAN_msg = frame.SerializeToString()
        rawmsg = struct.pack('<I' + str(len(serialized_pb_CAN_msg)
                                        ) + 's', PACK_HEAD, serialized_pb_CAN_msg)

        self._so.sendto(rawmsg, self._F4KAddress)

    def getdriverreturnvalue(self, paramtype='I'):
        (data, remoteaddr) = self._so.recvfrom(1024)

        (msgId, pbdata) = struct.unpack('<I' + str(len(data) - 4) + 's', data)
        if(0x00001019 == msgId):
            rxframe = CanFrame_pb2.CanFrame()
            try:
                rxframe.ParseFromString(pbdata)
            except:
                print('pbdata parse error')
                print(data)
                os.system('pause')
                return

            (value, b1, b2) = struct.unpack(
                '>' + paramtype + '2b', rxframe.Data)
            return value

    def querydrivervalue(self, valueindex, paramtype):
        while True:
            self.clearreadbuffer()
            self.senddrivercmd('r', valueindex, paramtype)
            try:
                ret = self.getdriverreturnvalue(paramtype)
            except socket.timeout:
                continue
            finally:
                return ret

    def writedrivervalue(self, valueindex, paramtype, value):
        while True:
            self.clearreadbuffer()
            self.senddrivercmd('w', valueindex, paramtype, value)
            try:
                ret = self.getdriverreturnvalue()
            except socket.timeout:
                continue
            finally:
                return ret

    def clearreadbuffer(self):
        while True:
            try:
                self._so.recvfrom(1024)
            except socket.timeout:
                break

    def readdriverobjectdict(self):
        global objectdict
        for key in objectdict:
            value = self.querydrivervalue(
                objectdict[key]['idx'], objectdict[key]['type'])
            objectdict[key]['value'] = value

        driverconfig = json.dumps(objectdict, indent=4, ensure_ascii=False)
        filename = './Driverconfig_' + \
            time.ctime().replace(' ', '_').replace(':', '_') + '.json'
        f = open(filename, 'w')
        f.write(driverconfig)
        f.close()
        print('Backup previous config file: ' + filename)

    def downloaddriverconfig(self, configfile):
        targetdict = json.load(open(configfile, 'r'))

        for key in targetdict:
            valueindex = targetdict[key]['idx']
            paramtype = targetdict[key]['type']
            value = targetdict[key]['value']

            ret = self.writedrivervalue(valueindex, paramtype, value)
            if ret != 0:
                print('return value error:' + str(ret))
        self.senddrivercmd('s', 0x01, 'I', 0)
        time.sleep(1)
        ret = self.getdriverreturnvalue()
        if ret != 0:
            print('return value error:' + str(ret))


configer = CRLDriverConfiger()
versionpass = configer.confirmgatewayversion()
if not versionpass:
    input('press enter to exit...')
    quit()
mailboxaddr = configer.addsingleIDlistenmailbox()
configer.readdriverobjectdict()
if(len(sys.argv) > 1):
    configfilename = sys.argv[1]
    configer.downloaddriverconfig(configfilename)
configer.deletelistenmailbox(mailboxaddr)
input('Operate finished, press enter to exit...')
