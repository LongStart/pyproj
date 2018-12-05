import matplotlib.pyplot as plt
from numpy.random import normal

ts = []
xs = []

x = 1
t = 0

t_step = 0.1
variance = t_step
mu = -0.01
sigma = -0.1

for t in range(0,5000):
    x += (mu * x * t_step +  sigma * x * normal(0, variance))
    t += t_step

    xs += [x]
    ts += [t]

plt.plot(ts, xs)
plt.show()