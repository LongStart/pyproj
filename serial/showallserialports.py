#!/usr/bin/env python
#-*- coding: utf-8 -*
import os
import serial
try:
    from serial import *
except AttributeError:
    print('no serial module found, install now...')
    os.system("pip install pyserial")
import serial.tools.list_ports

plist = list(serial.tools.list_ports.comports())

if len(plist) <= 0:
    print ("The Serial port can't find!")
else:
	for dev in plist:
		com_name = list(dev)[0]
		try:
			com_temp = serial.Serial(com_name)
		except SerialException:
			print(com_name + ' used')
		else:
			com_temp.close()
			print(com_name)
