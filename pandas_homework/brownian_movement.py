import matplotlib.pyplot as plt
from numpy.random import normal

ts = []
xs = []

x = 0
t = 0

t_step = 0.01
variance = t_step

for t in range(0,500):
    x += normal(0, variance)
    t += t_step

    xs += [x]
    ts += [t]

plt.plot(ts, xs)
plt.show()


