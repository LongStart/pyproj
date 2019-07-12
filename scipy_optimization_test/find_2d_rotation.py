import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares

def GenerateData(v1, v2, sigma=0.1, num=10):
    n_1 = v1 + np.random.normal(v1, sigma, (num,3))
    n_2 = v2 + np.random.normal(v2, sigma, (num,3))
    return n_1, n_2

def GenerateDataByRotVec(rotvec, num=10):
    # v_2 = np.random.random((num, 3))
    v_2 = np.array([[1,0]])
    rot = R.from_rotvec(rotvec)
    v_1 = rot.apply(v_2)
    return v_1, v_2

def hat(v):
    """
    :param v: 3x1 vector
    :return: 3x3 skew symmetric matrix
    """
    # yapf: disable
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])
    # yapf: enable

class Problem(object):
    def __init__(self, v1s, v2s):
        self.v1s = v1s
        self.v2s = v2s

    def f(self, x):
        rot = R.from_rotvec(x)
        return (rot.apply(self.v2s) - self.v1s).ravel()

    def jac(self, x):
        rot = R.from_rotvec(x)
        jacs = np.array([-rot.as_dcm().dot(hat(v)) for v in self.v2s])
        return jacs.reshape((len(jacs)*3, 3))

    def ljac(self, x):
        rot = R.from_rotvec(x)
        jacs = np.array([-hat(rot.apply(v)) for v in self.v2s])
        return jacs.reshape((len(jacs)*3, 3))

    

if __name__ == "__main__":

    # v2 = np.array([1,2,3])
    # rot = R.from_rotvec([3,4,2])
    # v1 = rot.apply(v2)
    v1, v2 = GenerateDataByRotVec([3,4,2], num=3)

    # v1s, v2s = GenerateData(v1, v2, num=5, sigma=1e-8)
    p = Problem(v1, v2)
    guess = [3.,4,2.]
    print(p.f(guess))
    print(p.jac(guess).transpose().dot(p.f(guess)) + p.f(guess).transpose().dot(p.jac(guess)))
    quit()
    res = least_squares(p.f, guess, jac=p.jac, method='lm',verbose=2, x_scale='jac')
    # res = least_squares(p.f, guess, method='lm',verbose=2)
    print(res.x)

    print('start')