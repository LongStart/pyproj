import matplotlib.pyplot as plt
from math import *

x = [0.1 *v for v in range(0,10)]
y = [v*v for v in x]
z = [exp(v) for v in x]

plt.plot(x, y)
plt.plot(x, z)
plt.ylabel('航向角 (°)')
plt.xlabel('时间 (s)')
plt.title('航向角控制')
plt.grid(True)
plt.legend(('航向角','俯仰角'))
plt.show()