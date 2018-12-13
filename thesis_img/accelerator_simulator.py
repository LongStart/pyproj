import matplotlib.pyplot as plt
from math import *
import numpy as np

dt = 0.02
ts = np.arange(0,100,dt)
thetas = np.array(pi/4 + pi/4.1 * np.sin (.4 * (ts-2) ))
ax_noise = np.random.normal(0.0,.01,len(ts))
az_noise = np.random.normal(0.0,.01,len(ts))

ax = np.sin(thetas) + ax_noise
az = np.cos(thetas) + az_noise
measure_thetas = np.arctan(ax / az)
# tan_y = np.tan(y * pi / 180 )
# y_noise = np.random.normal(0.0,1,len(y))
# tan_y += y_noise
# measure_y = np.arctan(tan_y) / pi * 180


plt.plot(ts, measure_thetas)
# plt.plot(ts, thetas)

plt.ylabel('角度 (°)')
plt.xlabel('时间 (s)')
plt.title('90°往复摆动')
plt.grid(True)
# plt.legend(('航向角','俯仰角'))
plt.show()