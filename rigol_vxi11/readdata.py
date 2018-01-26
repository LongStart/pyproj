import vxi11
import struct
import matplotlib.pyplot as plt

instr = vxi11.Instrument("192.168.192.88")

print(instr.ask("*IDN?"))
# print(instr.ask(':WAV:STAT?'))
instr.write(':STOP')
instr.write(':ACQ:MDEP 700000')
print(instr.ask(':ACQ:MDEP?'))
instr.write(':WAV:SOUR CHAN1')
instr.write(':WAV:MODE RAW')
# print(instr.ask(':WAV:MODE?'))

instr.write(':WAV:FORM WORD')
instr.write(':WAV:STAR 1')
instr.write(':WAV:STOP 125000')
print(instr.ask(':WAV:STOP?'))
data = instr.ask_raw(bytes(':WAV:DATA?', encoding='utf-8'))
datanum = int(data[2:11]) >> 1
print('datanum = %d' % datanum)
formatstr = '<' + datanum * 'H'
point = []
point = struct.unpack(formatstr, data[11:-1])
# print(point)
plt.plot(point)
plt.show()
# print(point)
# bdata = bytes(data[15:], encoding='utf-8')
# print(bdata.hex())
