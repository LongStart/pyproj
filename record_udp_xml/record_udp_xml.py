import socket
import time
import os
import sys
import re

local_addr = ('', 5555)
so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
so.bind(local_addr)

log_file_name = 'sensor_log_' + \
    time.strftime("%Y_%m_%d_%H_%M_%S", time.localtime()) + '.xml'
file_out = open(log_file_name, 'w')
xml_buff = b''

cnt = 0
sample_num = 10
if(len(sys.argv) > 1):
    sample_num = int(sys.argv[1])

print('record for: ' + str(sample_num) + ' data')

while True:
    data = so.recvfrom(1000)[0]
    xml_buff += data
    cnt += 1
    if(cnt >= sample_num):
        break

xml_str = re.sub('\<\?.*?\?\>', '\n', xml_buff.decode('utf-8'))
file_out.write(xml_str)
file_out.close()

