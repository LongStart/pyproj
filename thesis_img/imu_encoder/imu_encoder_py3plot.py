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
        # if((len(x1) > 0 and float(tt) != x1[-1]) or len(x1) == 0):
        x1 += [float(tt)]
        y1 += [float(dd)]

with open(imu_file) as fp:  
    for line in fp:
        (tt, yaw, pitch, roll) = line.split()
        x2 += [float(tt)]
        y2 += [float(pitch)+0.1]



plt.plot(x1, y1, '-')
plt.plot(x2, y2, '--')
plt.ylabel('俯仰角(°)')
plt.xlabel('Unix时间戳(s)')
plt.title('姿态传感器对比输出曲线')
plt.grid(True)
plt.legend(('编码器测量值','传感器滤波估计值'))
# plt.show()

plt.xlim(1544073813.35, 1544073819.9)
plt.ylim(-58, -18)
plt.savefig('encoder_imu_compare.tif', dpi=600)