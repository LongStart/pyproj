import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares

def f(x, p):
    return p[0] + p[1]* x + p[1]*p[2] * x**2

class Problem(object):
    def __init__(self, v1s, v2s):
        self.v1s = v1s
        self.v2s = v2s

    def f(self, p):
        return f(self.v1s, p) - self.v2s

    def jac(self, p):
        return np.vstack((np.ones(len(self.v1s)), x + p[2] * self.v1s**2, p[1] * self.v1s **2)).transpose()


def gauss_newton(problem, guess):
    x = guess
    for i in range(0, 30):
        j = problem.jac(x)
        b = -j.transpose().dot(problem.f(x))
        H = j.transpose().dot(j)
        update = np.linalg.solve(H, b)
        print('x: {}, update: {}'.format(x, update))
        # x = R.from_dcm( R.from_rotvec(x).as_dcm().dot(R.from_rotvec(update).as_dcm()) ).as_rotvec()
        x += update
    
        


if __name__ == "__main__":

    x = np.array([1,2,3,4,5,6])
    param = np.array([1,2,3])
    y = f(x, param)
    
    prob = Problem(x, y)
    guess = [3.5,4,2.]
    gauss_newton(prob, guess)
    # print(p.f(guess))
    # print(p.jac(guess).transpose().dot(p.f(guess)) + p.f(guess).transpose().dot(p.jac(guess)))
    quit()
    res = least_squares(prob.f, guess, jac=prob.jac, method='lm',verbose=2, x_scale='jac')
    # res = least_squares(p.f, guess, method='lm',verbose=2)
    print(res.x)

    print('start')