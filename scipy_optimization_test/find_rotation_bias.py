import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import sys

def GenerateNoiseData(v1, v2, sigma=0.00001):
    n_1 = v1 + np.random.normal(0, sigma, np.shape(v1))
    n_2 = v2 + np.random.normal(0, sigma, np.shape(v2))
    return n_1, n_2

def GenerateDataByRotVec(rotvec, num=10):
    v_2 = np.random.random((num, 3))
    # v_2 = np.array([[1,0,0],[0,1,0],[0,0,1]])
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

    @staticmethod
    def parse_space_vector(x):
        rotvec = np.array(x[0:3])
        bias_sum = np.array(x[3:6])
        return (rotvec, bias_sum)

    def numerical_jac(self, x):
        eps = 1e-4
        partial_d = []

        for i in range(3):
            rot = R.from_rotvec(x[0:3])
            d_r = np.array([0.,0,0])    
            d_r[i] += eps
            x_inc = np.hstack([(R.from_rotvec(d_r) * rot).as_rotvec(), x[3:]])
            partial_d.append((self.f(x_inc) - self.f(x))/eps)

        for i in range(3,len(x)):
            x_inc = np.array(x)
            x_inc[i] += eps
            partial_d.append((self.f(x_inc) - self.f(x))/eps)
        return np.vstack(partial_d).transpose()

    # residual (data_num x dim(3)) 
    def f(self, x):
        (rotvec, bias_sum) = Problem.parse_space_vector(x)
        rot = R.from_rotvec(rotvec)
        # print((rot.apply(self.v2s) - self.v1s - bias_sum).ravel())
        # quit()
        return (rot.apply(self.v2s) - self.v1s - bias_sum).ravel()
        # return 

    # jac (data_num x space_dim(9) x )
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


def gauss_newton(problem, guess):
    x = np.array(guess)
    for i in range(10):
        j = problem.numerical_jac(x)
        # j = problem.jac(x)
        b = -j.transpose().dot(problem.f(x))
        H = j.transpose().dot(j)
        update = np.linalg.solve(H, b)
        
        print('\nx: {}'.format(x))
        print('ud: {}'.format(update))
        print('dcost_dx: {}'.format(problem.d_cost_dx(x)))
        print('cost: {}'.format(problem.cost(x)))

        # print('j: {}'.format(j))
        (rotvec, bias_sum) = Problem.parse_space_vector(update)
        x[0:3] = (R.from_rotvec(rotvec) * R.from_rotvec(x[0:3])).as_rotvec()
        x[3:6] += bias_sum
    return x
        

if __name__ == "__main__":
    if len(sys.argv) > 1:
        static_test()
        quit()

    np.set_printoptions(precision=4, linewidth=np.inf)
    v1, v2 = GenerateDataByRotVec([0,1,1], num=1000)
    (v1, v2) = GenerateNoiseData(v1, v2)
    v1 += np.array([1.5,.0,.0])
    v2 += np.array([.0,.0,.0])

    p = Problem(v1, v2)
    
    guess = [1.,1.,1., 1,0,0]
    
    # print(p.jac(guess))
    # print(p.numerical_jac(guess))
    # quit()
    res = gauss_newton(p, guess)
    (rot, bias_sum) = Problem.parse_space_vector(res)
