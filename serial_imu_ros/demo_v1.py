import serial
from serial import *
import sys
import time
import os
import threading
from decode_bytearray import *



if(len(sys.argv) < 2):
    print("No input COM port, example: python.exe intboardpowermonitor.py COM5")
    input("Press enter to exit...")
    quit()
com_name = sys.argv[1]

com = serial.Serial(com_name, baudrate=115200, timeout=1)

buffer = bytearray(b'')

while(True):
    cmd = b''
    buff_len = len(buffer)
    if(buff_len > 5 and 0xa5 == buffer[0] and 0x5a == buffer[1] and buff_len >= (buffer[2] + 2) ):
        pack = decode_from_buffer(buffer[2:2+buffer[2]])
        buffer = buffer[buffer[2] + 2:]
        if(3 == len(pack)):
            print(pack)

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
