import matplotlib.pyplot as plt
from math import *
from scipy import interpolate
import numpy as np

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
        (tt, y, p, r) = line.split()
        x2 += [float(tt)]
        y2 += [float(p)+0.1]

x2 = np.array(x2)
y2 = np.array(y2)
x1 = np.array(x1)
y1 = np.array(y1)

npts = 10000
# x_idx_begin = 4500
# x_idx_end = 5500

x_idx_begin = 11800
x_idx_end = 15500

x = np.linspace(x1[x_idx_begin], x1[x_idx_end], npts)
dt = (x1[x_idx_end] - x1[x_idx_begin])/npts

print('dt: {}'.format(dt))

f1 = interpolate.interp1d(x1, y1, fill_value="extrapolate")
v1 = f1(x)

f2 = interpolate.interp1d(x2, y2, fill_value="extrapolate")
v2 = f2(x)

lags = np.arange(-npts + 1, npts)
ccov = np.correlate(v1 - v1.mean(), v2 - v2.mean(), mode='full')
ccor = ccov / (npts * v1.std() * v2.std())

# plt.plot(x, v1, '-')
# plt.plot(x, v2,'-')
lags = lags*dt
# plt.plot(lags, ccor, '-')


fig, axs = plt.subplots(nrows=2)

fig.subplots_adjust(hspace=0.4)
ax = axs[0]
ax.plot(x, v1, 'b', label='编码器输出角度曲线')
ax.plot(x, v2, 'r', label='姿态传感器滤波输出曲线')
ax.set_xlabel('Unix时间戳(s)')
ax.grid(1)
ax.legend(loc='upper right', fontsize='small', ncol=1)
ax = axs[1]
ax.plot(lags, ccor)
ax.set_ylabel('互相关系数')
ax.set_xlabel('时间(s)')
ax.grid(1)

plt.show()

plt.ylabel('互相关系数')
plt.xlabel('时间(s)')
plt.grid(True)
tar_x = lags[ccor.argmax()]
tar_y = max(ccor)
plt.plot(lags, ccor, linewidth=2)
plt.plot([tar_x, tar_x], [0,2],'r')
plt.show()

# plt.savefig('encoder_imu_compare.pdf')