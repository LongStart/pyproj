import matplotlib.pyplot as plt
from math import *
import numpy as np

# phis = []
# thetas = []
r = 1
d = 4
R = 2
theta = np.arange(-5*pi,5*pi,0.1)
phi = np.arctan2(np.sin(theta), d/R - np.cos(theta))
# theta = np.arccos((R-d*np.cos(phi))/R) + phi
theta /= (2*pi)

plt.plot(theta, phi,  '-.')

# plt.ylabel('角度(°)')
# plt.xlabel('时间 (秒)')
# plt.title('欧拉角姿态输出')
plt.grid(True)
# plt.legend(('偏航角','俯仰角','横滚角'))
plt.show()