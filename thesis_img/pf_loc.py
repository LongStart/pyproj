import matplotlib.pyplot as plt
from math import *

# x = [0.1 *v for v in range(0,10)]
# y = [v*v for v in x]
# z = [exp(v) for v in x]
x1 = []
y1 = []
x2 = []
y2 = []
filepath = 'pf_loc_2018_12_01.txt'  
with open(filepath) as fp:  
    for line in fp:
        (xx1, yy1, xx2, yy2) = line.split()
        x1 += [float(xx1)]
        y1 += [float(yy1)]
        x2 += [float(xx2)]
        y2 += [float(yy2)]

           

plt.plot(x1, y1, '-.')
plt.plot(x2, y2, '.')
plt.ylabel('纵坐标(米)')
plt.xlabel('横坐标 (米)')
plt.title('定位跟踪轨迹')
plt.grid(True)
plt.legend(('真实轨迹','定位轨迹'))
plt.axis('equal')
# plt.show()
plt.savefig('pf_loc.pdf')