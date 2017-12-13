#!/usr/bin/env python
#-*- coding: utf-8 -*
import os
from serial import *
import struct
import serial.tools.list_ports

def finduartconsoleport():
	plist = list(serial.tools.list_ports.comports())

	if len(plist) <= 0:
		print ("The Serial port can't find!")
		input()
		quit()

	UNAME_PACK_HAED = 0x1032
	targetport = ''
	for dev in plist:
		com_name = list(dev)[0]
		try:
			com_temp = serial.Serial(com_name, baudrate=115200, timeout=0.01)
		except SerialException:
			# print(com_name + ' used')
			pass
		else:
			com_temp.write(struct.pack('<2I', UNAME_PACK_HAED, 0))
			databack = com_temp.read(12)
			if b'SeerDIOBoard' == databack:
				targetport = com_name
				com_temp.close()
				break
			else:
				com_temp.close()
				# print(com_name + ' not connected')
				continue

	# if(targetport != ''):
	# 	print('Target port is: ' + targetport)
	# else:
	# 	print('cannot find target port')
	
	return targetport

print(finduartconsoleport())