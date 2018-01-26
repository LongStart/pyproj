import vxi11
import struct
import matplotlib.pyplot as plt

instr = vxi11.Instrument("192.168.192.88")
print(instr.ask("*IDN?"))
instr.close()

instr = vxi11.Instrument("192.168.192.88")
print(instr.ask("*IDN?"))
instr.write(':WAV:FORM BYTE')
print(instr.ask(':TIM:MODE?'))
print(instr.ask(':CHAN1:DISP?'))
print(instr.ask(':CHAN2:DISP?'))
print(instr.ask(':CHAN1:OFFS?'))
print(instr.ask(':CHAN1:SCAL?'))
print(instr.ask(':CHAN2:OFFS?'))
print(instr.ask(':CHAN2:SCAL?'))

print(instr.ask(':TIM:OFFS?'))
print(instr.ask(':TIM:SCAL?'))

print(instr.ask(':CHAN1:COUP?'))
print(instr.ask(':CHAN1:BWL?'))
print(instr.ask(':CHAN1:PROB?'))
print(instr.ask(':CHAN1:IMP?'))
print(instr.ask(':CHAN1:INV?'))
print(instr.ask(':CHAN1:UNIT?'))

print(instr.ask(':CHAN2:COUP?'))
print(instr.ask(':CHAN2:BWL?'))
print(instr.ask(':CHAN2:PROB?'))
print(instr.ask(':CHAN2:IMP?'))
print(instr.ask(':CHAN2:INV?'))
print(instr.ask(':CHAN2:UNIT?'))

print(instr.ask(':ACQ:MDEP?'))
print(instr.ask(':ACQ:SRAT?'))

print(instr.ask(':TRIG:STAT?'))
print(instr.ask(':TRIG:STAT?'))
print(instr.ask(':TRIG:STAT?'))
print(instr.ask(':TRIG:SWE?'))
print(instr.ask(':TRIG:MODE?'))

print(instr.ask(':TRIG:EDGE:SOUR?'))
print(instr.ask(':TRIG:EDGE:LEV?'))
print(instr.ask(':TRIG:EDGE:SLOP?'))

instr.write(':WAV:MODE NORM')
print(instr.ask(':WAV:PRE?'))
instr.write(':WAV:STAR 1')
instr.write(':WAV:STOP 0')
instr.write(':RUN')
