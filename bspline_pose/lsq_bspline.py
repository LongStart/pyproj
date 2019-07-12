import numpy as np

x = np.linspace(-15, 15, 200)
y = np.exp(-x**2) + 0.1 * np.random.randn(200)

from scipy.interpolate import make_lsq_spline, BSpline
t = [-1, 0, 1]
k = 4
t = np.r_[(x[0],)*(k+1),
          t,
          (x[-1],)*(k+1)]
print(t)
spl = make_lsq_spline(x, y, t, k)

from scipy.interpolate import make_interp_spline
spl_i = make_interp_spline(x, y)

import matplotlib.pyplot as plt
# xs = np.linspace(-3, 3, 100)
xs = x
plt.plot(x, y, 'ro', ms=5)
plt.plot(xs, spl(xs), 'g-', lw=3, label='LSQ spline')
plt.plot(xs, spl_i(xs), 'b-', lw=3, alpha=0.7, label='interp spline')
plt.legend(loc='best')
plt.show()