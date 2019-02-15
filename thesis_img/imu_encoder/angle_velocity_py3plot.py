import matplotlib.pyplot as plt
from math import *
t1 = []
y1 = []
t2 = []
gxs = []
gys = []
gzs = []

t3 = []
v3 = []

encoder_file = 'build/encoder.txt'  
imu_file = 'build/gyro_acc_adc.txt'  

line_num = 500
sample_period = 1e-2
rad_per_count = 2. * pi / line_num / 4 


with open(encoder_file) as fp:  
    for line in fp:
        (tt, dd) = line.split()
        # angle = float(dd)
        # delta_angle = angle - y1[-1]
        t1 += [float(tt)]
        y1 += [float(dd)]

        # if(t1[-1] != t3[-1]):
        #     t3 += [t1[-1]]
        #     v3 += [delta_angle / 20.]
        #     line_count = 0
        #     t_prev = t1[-1]
        # else:
        #     line_count += 1

t3 += [t1[0]]
v3 += [0]
prev_v1 = 0
for i in range (0, len(t1)):
    if(t1[i] != t3[-1]):
        # print('.')
        t3 += [t1[i]]
        delta_v = y1[i] - prev_v1
        v3 += [delta_v*20]
        prev_v1 = y1[i]

with open(imu_file) as fp:  
    for line in fp:
        (tt, gx, gy, gz, ax, ay, az) = line.split()
        t2 += [float(tt)]
        gxs += [float(gx)]
        gys += [float(gy)]
        gzs += [float(gz)]


plt.plot(t3, v3, '-')
# plt.plot(t1, y1, '.-')
# plt.plot(t2, gxs, '-')
plt.plot(t2, gys, '-')
# plt.plot(t2, gzs, '-')

plt.ylabel('角速度(rad/s)')
plt.xlabel('Unix时间戳(s)')
plt.title('姿态传感器对比输出曲线')
plt.grid(True)
plt.legend(('编码器测量值','传感器原始数值'))
# plt.xlim(1544073813.35, 1544073819.9)
# plt.ylim(-58, -18)

# plt.xlim(1545637.35, 1544073819.9)
# plt.ylim(-58, -18)
plt.show()
# plt.savefig('encoder_imu_compare.pdf')