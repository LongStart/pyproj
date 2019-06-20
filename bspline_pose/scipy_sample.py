from scipy.interpolate import BSpline
import numpy as np
k = 4
t = [0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1]
x = [0, 1, 2, 3, 4, 5]
y = [1, 0, 1, 0, 1, 0]
spl_x = BSpline(t, x, k)
spl_y = BSpline(t, y, k)
# spl(2.5)

# bspline(2.5, t, c, k)

import matplotlib.pyplot as plt
fig, ax = plt.subplots()
xx = np.linspace(0, 10, 50)
# ax.plot(xx, [BSpline.bspline(x, t, c ,k) for x in xx], 'r-', lw=3, label='naive')
ax.plot(spl_x(xx), spl_y(xx), 'b-', lw=4, alpha=0.7, label='BSpline')
ax.plot(x, y, 'x')
ax.grid(True)
ax.legend(loc='best')
plt.show()