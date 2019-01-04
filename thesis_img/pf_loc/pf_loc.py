import matplotlib.pyplot as plt
from math import *

# x = [0.1 *v for v in range(0,10)]
# y = [v*v for v in x]
# z = [exp(v) for v in x]
x1 = []
y1 = []
x2 = []
y2 = []
xp = []
yp = []
t1 = []
tp = []
step = 0.05
t = 0
filepath = 'pf_loc_2018_12_26_fast.txt'  
with open(filepath) as fp:  
    for line in fp:
        # (xx1, yy1, xx2, yy2) = line.split()
        # x1 += [float(xx1)]
        # y1 += [float(yy1)]
        # x2 += [float(xx2)]
        # y2 += [float(yy2)]
        
        points = [float(v) for v in line.split()]
        x1 += [points[0]]
        y1 += [points[1]]
        x2 += [points[2]]
        y2 += [points[3]]
        t += step
        t1 += [t]
        for i in range(4,len(points)):
            if(0 == i % 2):
                xp += [points[i]]
                tp += [t]
            else:
                yp += [points[i]]

# plt.plot(tp, xp, 'b.',alpha=0.3,markersize=5,markeredgecolor='none')
# plt.plot(t1, x2, 'k-')
# plt.plot(t1, x1, 'r-')

# plt.grid(True)
# plt.show()

fig, axs = plt.subplots(nrows=2)
fig.subplots_adjust(hspace=0.4)
ax = axs[0]
ax.grid(True)
ax.plot(tp, xp, 'b.', alpha=0.3,markeredgecolor='none',label='粒子位置')
ax.plot(t1, x1, 'r', label='真实位置',linewidth=2)
ax.plot(t1, x2, 'w', label='估计位置',linewidth=1)
ax.set_ylabel('横坐标(米)')
ax.set_xlabel('时间(秒)')


ax = axs[1]
ax.plot(tp, yp, 'b.', alpha=0.3,markeredgecolor='none',label='粒子位置')
ax.plot(t1, y1, 'r', label='真实位置',linewidth=2)
ax.plot(t1, y2, 'w', label='估计位置',linewidth=1)

ax.grid(True)
ax.legend(loc='lower right', fontsize='small', ncol=1)
ax.set_ylabel('纵坐标(米)')
ax.set_xlabel('时间(秒)')

plt.show()