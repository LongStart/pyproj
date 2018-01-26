import socket
import struct
from time import sleep
import time
import sys
import message_navigation_pb2
import message_odometer_pb2
import message_header_pb2
from configchassis import configf4kernelchassis
import threading
import matplotlib.pyplot as plt

global is_exit
global odom
global vcmd
is_exit = False
vcmd = []
odom = []

F4kCommandPort = 15003
F4kAddr = ('192.168.192.4', F4kCommandPort)
F4kAddr_old = ('192.168.192.4', 5003)
PACK_HEAD = 0x00001034

model_file = '.\\robot.model'
if(len(sys.argv) > 1):
    model_file = sys.argv[1]
    tail = model_file[-6:]
    if(tail != '.model'):
        print('invalid file format:' + tail + ',should be *.model')
        input()
        quit()

configf4kernelchassis(model_file)


sendcnt = 100


def exit_handler():
    global is_exit
    is_exit = True
    sys.exit()


def heartbeatthreadfunc():
    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    heartbeatMsg = struct.pack('<2I', 0x200, 0xffffffff)

    so.sendto(heartbeatMsg, F4kAddr_old)
    configf4kernelchassis(model_file)

    global is_exit
    while True:
        sleep(1)
        so.sendto(heartbeatMsg, F4kAddr_old)
        if(is_exit):
            quit()


def getodomthreadfunc():
    print('get odom thread start...')
    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    localaddr = ('', 5002)
    so.bind(localaddr)
    so.settimeout(0.5)
    global odom
    global is_exit
    while True:
        try:
            data = so.recvfrom(1024)[0]
        except socket.timeout:
            continue

        frame = message_odometer_pb2.Message_Odometer()
        (msgID, pbdata) = struct.unpack('<I' + str(len(data) - 4) + 's', data)
        if(msgID == 0x00):
            frame.ParseFromString(pbdata)
            odom += [(frame.header.data_nsec, frame.x,
                      frame.y, frame.angle, frame.is_stop)]
        if(is_exit):
            print('get odom thread exit')
            print(odom)
            sys.stdout.flush()
            quit()


def sendspeed(vx):
    chassisCmd = message_navigation_pb2.Message_NavSpeed()
    chassisCmd.x = 0
    chassisCmd.y = 0
    chassisCmd.rotate = 0.0
    chassisCmd.steer_angle = 0.0

    serializedChassisCmd = chassisCmd.SerializeToString()

    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.settimeout(0.03)

    msg = struct.pack('<I' + str(len(serializedChassisCmd)) + 's',
                      PACK_HEAD, serializedChassisCmd)

    so.sendto(msg, F4kAddr)


def plotcurve(speedcmd_list, odometer_list):
    time_stamp_offset_ns = float(odometer_list[1][0])
    print('time offset = %dns' % time_stamp_offset_ns)

    cmd_vx_mps = []
    cmd_t_ms = []
    for cmd in speedcmd_list:
        tempT = (float(cmd[1]) - time_stamp_offset_ns) / 1000000
        if(tempT < -1e10):
            print('invalid command point:' + str(cmd))
            continue
        cmd_t_ms += [tempT]
        cmd_vx_mps += [float(cmd[0])]

    t_ms = []
    x_m = []
    y_m = []
    theta_rad = []
    vx_mps = [0]
    vy_mps = [0]
    v_mps = [0]
    for v in odometer_list:
        tempT = (float(v[0]) - time_stamp_offset_ns) / 1000000
        if(tempT < -1e10):
            print('invalid odom point:' + str(v))
            continue
        t_ms += [tempT]
        x_m += [float(v[1])]
        y_m += [float(v[2])]
        if(len(x_m) > 1):
            vx_mps += [(x_m[-1] - x_m[-2]) / (t_ms[-1] - t_ms[-2]) * 1000]
            vy_mps += [(y_m[-1] - y_m[-2]) / (t_ms[-1] - t_ms[-2]) * 1000]
            v_mps += [(vx_mps[-1]**2 + vy_mps[-1]**2)**0.5]
            if vx_mps[-1] < 0:
                v_mps[-1] *= -1
    # print(t_ms)
    # print(speedcmd_list)
    # plt.plot(t_ms, x_m)
    # plt.plot(t_ms, v_mps)
    # plt.plot(t_ms, vx_mps)
    # plt.plot(x_m, y_m)
    # plt.plot(t_ms, x_m, t_ms, y_m)
    # plt.plot(t_ms, x_m, t_ms, v_mps)
    # plt.plot(cmd_t_ms, cmd_vx_mps)
    plt.plot(t_ms, vx_mps, cmd_t_ms, cmd_vx_mps, '.')
    plt.show()


heartbeatThread = threading.Thread(target=heartbeatthreadfunc)
heartbeatThread.setDaemon(True)
heartbeatThread.start()

getodomThread = threading.Thread(target=getodomthreadfunc)
getodomThread.setDaemon(True)
getodomThread.start()

delta_t = 0.02

while True:
    try:
        sleep(delta_t)
    except KeyboardInterrupt:
        plotcurve(vcmd, odom)
        sys.stdout.flush()
        exit_handler()

        exit()

    vx = 1
    sendspeed(vx)
    vcmd += [(vx, time.time() * 1000000000)]
