from geomdl import BSpline
import matplotlib.pyplot as plt
import numpy as np

# len(knot_vector) == degree + num_ctrlpts + 1

# Create a 3-dimensional B-spline Curve
curve = BSpline.Curve()

# Set degree
curve.degree = 4

# Set control points
curve.ctrlpts = [[10, 5, 10], [10, 20, -30], [40, 10, 25], [10, 50, 25], [-10, 5, 0]]
# curve.ctrlpts = [[10, 5, 10], [10, 20, -30], [40, 10, 25], [-10, 5, 0]]

# Set knot vector
curve.knotvector = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1]

# Set evaluation delta (controls the number of curve points)
curve.delta = 0.05

# Get curve points (the curve will be automatically evaluated)
curve_points = np.array(curve.evalpts)
ctrl_points = np.array(curve.ctrlpts)

# print(curve_points)
plt.plot(curve_points[:,0], curve_points[:,1])
plt.plot(ctrl_points[:, 0], ctrl_points[:, 1], 'o')
plt.show()