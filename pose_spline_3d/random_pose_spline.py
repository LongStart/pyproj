import bsplines
import numpy as np
import sm
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == "__main__":
    rv = sm.RotationVector()
    bsp = bsplines.BSplinePose(4, rv)
    # bsp2 = bsplines.BSplinePose(4, rv)
    curve = []
    num = 5
    for i in range(num):
        curve.append(numpy.random.random(6) - 0.5)
    curve = np.vstack(curve).T

    times = np.linspace(0, 2, num)
    bsp.initPoseSplineSparse(times, curve, num - 3, 1e-5)
    # bsp2.initPoseSplineSparse(np.linspace(0, 2, num), curve, num - 3, 1e-5)

    control_points = curve[0:3]
    positions = np.vstack([bsp.position(p) for p in np.linspace(0, 2, 200)])
    # positions2 = np.vstack([bsp2.position(p) for p in np.linspace(0, 2, 200)])

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect('equal')
    ax.plot(*positions.T)
    # ax.plot(*positions2.T)
    ax.scatter(*control_points)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()