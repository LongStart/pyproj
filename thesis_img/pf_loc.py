import matplotlib.pyplot as plt
from math import *

# x = [0.1 *v for v in range(0,10)]
# y = [v*v for v in x]
# z = [exp(v) for v in x]
x1 = []
y1 = []
x2 = []
y2 = []
filepath = '2018_12_01.txt'  
with open(filepath) as fp:  
    for line in fp:
        (xx1, yy1, xx2, yy2) = line.split()
        x1 += [float(xx1)]
        y1 += [float(yy1)]
        x2 += [float(xx2)]
        y2 += [float(yy2)]

           

plt.plot(x1, y1, '-.')
plt.plot(x2, y2, '.')
plt.ylabel('航向角 (°)')
plt.xlabel('时间 (s)')
plt.title('航向角控制')
plt.grid(True)
plt.legend(('航向角','俯仰角'))
plt.axis('equal')
plt.show()