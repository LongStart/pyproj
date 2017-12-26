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

cmddict = {
    1: "Controller get wake up type",
    2: "Launch key was pressed",
    3: "Shutdown key was pressed",
    4: "PC shutdown detected and may be shutdown from desktop",
    5: "Shutdown key was released after PC shutdown, shutdown source: desktop",
    6: "Shutdown key was released during PC shutting down",
    7: "Shutdown key keep pressed during PC shutting down",
    8: "PC was shutdown by key input",
    9: "Shutdown timer timeup, power cut up!",
    10: "Shutdown key released after PC shutdown, shutdown source: key",
    16: "Heart beat... State machine: Uninit",
    17: "Heart beat... State machine: Standby",
    18: "Heart beat... State machine: Launch key pressing",
    19: "Heart beat... State machine: Normal working",
    20: "Heart beat... State machine: Shutting down"}


com = serial.Serial(com_name, baudrate=115200, timeout=0.6)
try:
    os.mkdir('.\\log')
except FileExistsError:
    pass

log_file_name = '.\\log\\powerlog_' + \
    time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) + '.log'
file_out = open(log_file_name, 'w')

# file_out.write('Intboard power monitor start...\n')
# file_out.write('Logfile name: ' + 'log_file_name\n')
# file_out.write('Monitor version: V1.0.0\n')

file_out.close()


def loginfo(info):
    file_out = open(log_file_name, 'a')
    info = '[' + time.ctime() + ']' + info
    file_out.write(info + '\n')
    file_out.close()
    print(info)


def daemonthreadfunc():
    loginfo('Main thread killed...')
    time.sleep(1)


loginfo('Intboard power monitor start...\n')
loginfo('Logfile name: ' + log_file_name + '\n')
loginfo('Monitor version: V1.0.0\n')

while(True):
    cmd = b''
    try:
        cmd = com.read(1)
    except KeyboardInterrupt:
        loginfo('Monitor closed')
        break

    if(cmd == b''):
        loginfo('Heart beat broken!')
        continue

    logstr = 'Raw command: 0x' + \
        hex(cmd[0]).replace('0x', '').zfill(2) + ' - ' + cmddict[cmd[0]]
    loginfo(logstr)


# file_out.close()
com.close()
