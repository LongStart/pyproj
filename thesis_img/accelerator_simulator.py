import matplotlib.pyplot as plt
from math import *
import numpy as np

data_file = '../read_mouse/finger_data/data_04.txt'
raw_datas = []
ts = []
t = 0
dt = 0.02
with open(data_file) as fp:  
    for line in fp:
        raw_datas += [float(line.split()[0])]
        t += dt
        ts += [t]

thetas = np.array([v * 90 / 900 for v in raw_datas]) - 10
noise = np.random.normal(0.,10,len(thetas))
thetas += noise

plt.plot(ts, thetas, linewidth=0.5)

plt.ylabel('角度 (°)')
plt.xlabel('时间 (s)')
plt.xlim(13, 32)
plt.ylim(0, 82)
plt.grid(True)

plt.show()