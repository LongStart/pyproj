import matplotlib.pyplot as plt
from numpy.random import normal

ts = []
ss = []
xs = []

x = 1
s = 0
t = 0

t_step = 0.01
variance = t_step

alpha = .1
v = .1
a = .1
sigma_11 = 0.5
sigma_21 = 0.5
sigma_22 = 0.5
mu = 0.1


for t in range(0,5000):
    dbt = normal(0, variance)
    dwt = normal(0, variance)
    prev_x = x
    x += alpha * (v - prev_x) * t_step + sigma_11 * dbt
    s += mu * (prev_x - a * s) * t_step + sigma_21 * dbt + sigma_22 * dwt
    t += t_step

    xs += [x]
    ts += [t]
    ss += [s]

# plt.plot(xs, ss)
plt.plot(ts, xs)
plt.plot(ts, ss)
plt.show()