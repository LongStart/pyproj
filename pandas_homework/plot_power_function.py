import matplotlib.pyplot as plt
from math import *

x = [0.01 *v for v in range(20,500)]
y = [(1-.95/(2**20 * v ** 20)) for v in x]

plt.plot(x, y)
# plt.plot(x, z)
plt.ylabel('航向角 (°)')
plt.xlabel('时间 (s)')
plt.title('航向角控制')
plt.grid(True)
plt.legend(('航向角'))
plt.show()