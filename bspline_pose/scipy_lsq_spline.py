import numpy as np

data_num = 60
x = np.linspace(-3, 3, data_num)
y = np.exp(-x**2) + 0.1 * np.random.randn(data_num)

from scipy.interpolate import make_lsq_spline, BSpline
# t = [-1, 0, 1]
t = np.linspace(-2.7, 2.7, 50)
k = 3
t = np.r_[(x[0],)*(k+1),
          t,
          (x[-1],)*(k+1)]
spl = make_lsq_spline(x, y, t, k)

from scipy.interpolate import make_interp_spline
spl_i = make_interp_spline(x, y)

import matplotlib.pyplot as plt
xs = np.linspace(-3, 3, 500)
plt.plot(x, y, 'ro', ms=5)
plt.plot(xs, spl(xs), 'g-', lw=3, label='LSQ spline')
plt.plot(xs, spl_i(xs), 'b-', lw=3, alpha=0.7, label='interp spline')
plt.legend(loc='best')
plt.show()