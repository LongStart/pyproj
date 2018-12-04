import matplotlib.pyplot as plt
from math import *

# x = [0.1 *v for v in range(0,10)]
# y = [v*v for v in x]
# z = [exp(v) for v in x]
ts = []
yaws = []
pitchs = []
rolls = []

filepath = 'imu_data_1114.txt'  
t_step = 0.06
t = 0
with open(filepath) as fp:  
    for line in fp:
        (yaw, pitch, roll) = line.split()
        t += t_step
        yaws += [float(yaw)]
        pitchs += [float(pitch)]
        rolls += [float(roll)]
        ts += [t]

           

plt.plot(ts, yaws,'-')
plt.plot(ts, pitchs,'--')
plt.plot(ts, rolls, '-.')

plt.ylabel('角度(°)')
plt.xlabel('时间 (秒)')
plt.title('欧拉角姿态输出')
plt.grid(True)
plt.legend(('偏航角','俯仰角','横滚角'))
plt.show()