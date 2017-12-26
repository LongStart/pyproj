import socket
import struct
from time import sleep
import sys
import message_navigation_pb2
from configchassis import configf4kernelchassis

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

chassisCmd = message_navigation_pb2.Message_NavSpeed()
chassisCmd.x = 0.00
chassisCmd.y = 0
chassisCmd.rotate = -0.2
chassisCmd.steer_angle = 0.0

serializedChassisCmd = chassisCmd.SerializeToString()

so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
so.settimeout(0.03)

msg = struct.pack('<I' + str(len(serializedChassisCmd)) + 's',
                  PACK_HEAD, serializedChassisCmd)

sendcnt = 100
heartbeatMsg = struct.pack('<2I', 0x200, 0xffffffff)
while sendcnt > 0:
    try:
        sleep(0.02)
    except KeyboardInterrupt:
        so.close()
        exit()
    so.sendto(msg, F4kAddr)
    so.sendto(heartbeatMsg, F4kAddr_old)
    sendcnt -= 1
    print('.')
    sys.stdout.flush()

so.close()
