import matplotlib.pyplot as plt
from math import *
x1 = []
y1 = []
x2 = []
y2 = []
encoder_file = 'build/encoder.txt'  
imu_file = 'build/imu.txt'  

with open(encoder_file) as fp:  
    for line in fp:
        (tt, dd) = line.split()
        x1 += [float(tt)]
        y1 += [float(dd)]

with open(imu_file) as fp:  
    for line in fp:
        (tt, dd) = line.split()
        x2 += [float(tt)]
        y2 += [float(dd)]

           

plt.plot(x1, y1, '-')
plt.plot(x2, y2, '--')
plt.ylabel('俯仰角(°)')
plt.xlabel('时间(秒)')
plt.title('姿态传感器对比输出曲线')
plt.grid(True)
plt.legend(('编码器测量值','传感器滤波估计值'))
# plt.axis('equal')
plt.show()