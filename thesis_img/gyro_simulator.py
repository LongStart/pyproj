import matplotlib.pyplot as plt
from math import *
import numpy as np

dt = 0.02
x = np.arange(0,100,dt)
y = np.array(45 + 45 * np.sin (4 * (x-2) ))
dy = np.diff(y)
dy /= dt
y_noise = np.random.normal(0.0,70,len(dy))
dy += y_noise
int_y = np.cumsum(dy)
int_y *= dt
int_y += 45

# plt.plot(x,y)
# plt.plot(x[1:],dy)
plt.plot(x[1:], int_y)
plt.ylabel('角度 (°)')
plt.xlabel('时间 (s)')
plt.title('90°往复摆动')
plt.grid(True)
plt.show()