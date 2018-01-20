import vxi11
import struct
import matplotlib.pyplot as plt
import time
import sys

instr = vxi11.Instrument("192.168.192.88")
print(instr.ask("*IDN?"))
instr.write(':STOP')
instr.write(':WAV:SOUR CHAN1')
instr.write(':WAV:MODE RAW')
instr.write(':WAV:FORM BYTE')
instr.write(':WAV:STAR 1')
instr.write(':WAV:STOP 7000000')
instr.write(':WAV:RES')
instr.write(':WAV:BEG')

filename = './waveform_' + \
    time.ctime().replace(' ', '_').replace(':', '_') + '.txt'

f = open(filename, 'w')
cnt = 0

while True:
    state = instr.ask(':WAV:STAT?')
    data = instr.ask_raw(bytes(':WAV:DATA?', encoding='utf-8'))
    datanum = int(data[2:11])
    print('datanum = %d' % datanum)
    formatstr = '<' + datanum * 'B'
    point = []
    point = struct.unpack(formatstr, data[11:-1])
    # print(point)
    for val in point:
        f.write('t: %d, ch: %d\n' % (cnt, val))
        sys.stdout.write('.')
        cnt = cnt + 1

    if(state[0:4] == 'READ'):
        continue
    elif state[0:4] == 'IDLE':
        instr.write(':WAV:END')
        break
    else:
        print('state = ' + state)
        break

f.close()
