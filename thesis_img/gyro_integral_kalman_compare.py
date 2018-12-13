import matplotlib.pyplot as plt
from math import *
import numpy as np

data_file = '../read_mouse/finger_data/data_01.txt'
raw_datas = []
ts = []
t = 0
dt = 0.02
with open(data_file) as fp:  
    for line in fp:
        raw_datas += [float(line.split()[0])]
        t += dt
        ts += [t]

thetas = np.array([v * 90 / 550 for v in raw_datas])
d_thetas = np.diff(thetas) / dt
noise = np.random.normal(0.01,80,len(d_thetas))
d_thetas += noise
measure_thetas = np.cumsum(d_thetas) * dt + thetas[0]

plt.plot(ts[1:], measure_thetas,'-.')
plt.plot(ts, thetas,'-')

plt.ylabel('角度 (°)')
plt.xlabel('时间 (s)')
plt.title('90°往复摆动')
plt.grid(True)
plt.legend(('陀螺仪积分输出','卡尔曼滤波器输出'))
plt.show()