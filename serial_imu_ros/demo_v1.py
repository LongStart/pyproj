import serial
from serial import *
import sys
import time
import os
import threading



if(len(sys.argv) < 2):
    print("No input COM port, example: python.exe intboardpowermonitor.py COM5")
    input("Press enter to exit...")
    quit()
com_name = sys.argv[1]

com = serial.Serial(com_name, baudrate=115200, timeout=1)

buffer = bytearray(b'')

while(True):
    cmd = b''
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
