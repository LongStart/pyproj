import socket
import struct
import sys

F4kCommandPort = 15003
F4kAddr = ('192.168.192.4', F4kCommandPort)


def f4kernelcommand(packHead, arg):
    if arg == []:
        arg = [0xffffffff]
    msg = struct.pack('<' + str(len(arg) + 1) + 'I', packHead, *arg)
    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.settimeout(0.05)

    so.sendto(msg, F4kAddr)
    so.close()


def f4kernelquery(packHead, arg):
    if arg == []:
        arg = [0xffffffff]
    msg = struct.pack('<' + str(len(arg) + 1) + 'I', packHead, *arg)
    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.settimeout(0.5)

    so.sendto(msg, F4kAddr)
    ret = so.recvfrom(1024)[0]
    return ret
    so.close()


def liftbelt(height):
    msg = struct.pack('<2If', 0x00001049, 1, height)
    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.settimeout(0.05)

    so.sendto(msg, F4kAddr)
    so.close()


def setpulse(count):
    msg = struct.pack('<3Ii', 0x00001029, 1, 0, count)
    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.settimeout(0.05)

    so.sendto(msg, F4kAddr)
    so.close()
