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

        (rotvec, bias_sum) = Problem.parse_space_vector(update)
        x[0:3] = (R.from_rotvec(rotvec) * R.from_rotvec(x[0:3])).as_rotvec()
        x[3:6] += bias_sum
    return x

class Problem(object):
    def __init__(self, v1s, v2s):
        self.v1s = v1s.transpose()
        self.v2s = v2s.transpose()

    @staticmethod
    def parse_space_vector(x):
        rotvec = np.array(x[0:3])
        bias_sum = np.array(x[3:6])
        return (rotvec, bias_sum)

    def f(self, x):
        (rotvec, bias_sum) = Problem.parse_space_vector(x)
        rot = R.from_rotvec(rotvec)
        return (rot.apply(self.v2s) - self.v1s - bias_sum).ravel()

    def jac(self, x):
        (rotvec, bias_sum) = Problem.parse_space_vector(x)
        rot = R.from_rotvec(rotvec)
        jacs = np.vstack([
            np.hstack([-hat(rot.apply(v)), -np.eye(3)]) 
            for v in self.v2s])
        return jacs

    def d_cost_dx(self, x):
        return self.jac(x).transpose().dot(self.f(x)) + self.f(x).transpose().dot(self.jac(x))

    def cost(self, x):
        return self.f(x).transpose().dot(self.f(x))