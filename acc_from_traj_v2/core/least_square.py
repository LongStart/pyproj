import numpy as np
from scipy.spatial.transform import Rotation as R
from lie_algebra import *

# def hat(v):
#     """
#     :param v: 3x1 vector
#     :return: 3x3 skew symmetric matrix
#     """
#     # yapf: disable
#     return np.array([[0.0, -v[2], v[1]],
#                      [v[2], 0.0, -v[0]],
#                      [-v[1], v[0], 0.0]])
#     # yapf: enable

def gauss_newton(problem, guess, step=20):
    x = guess
    for i in range(step):
        j = problem.jac(x)
        b = -j.transpose().dot(problem.f(x))
        H = j.transpose().dot(j)
        update = np.linalg.solve(H, b)
        print('x: {}, ud: {}, dcost_dx:{}, cost: {}'.format(x, update, problem.d_cost_dx(x), problem.cost(x)))
        x = (R.from_rotvec(update) * R.from_rotvec(x)).as_rotvec()
    return x

class Problem(object):
    def __init__(self, v1s, v2s):
        self.v1s = v1s.transpose()
        self.v2s = v2s.transpose()

    def f(self, x):
        rot = R.from_rotvec(x)
        return (rot.apply(self.v2s) - self.v1s).ravel()

    def jac(self, x):
        rot = R.from_rotvec(x)
        jacs = np.vstack([-hat(rot.apply(v)) for v in self.v2s])
        return jacs

    def d_cost_dx(self, x):
        return self.jac(x).transpose().dot(self.f(x)) + self.f(x).transpose().dot(self.jac(x))

    def cost(self, x):
        return self.f(x).transpose().dot(self.f(x))