import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import sys

def GenerateData(v1, v2, sigma=0.1, num=10):
    n_1 = v1 + np.random.normal(v1, sigma, (num,3))
    n_2 = v2 + np.random.normal(v2, sigma, (num,3))
    return n_1, n_2

def GenerateDataByRotVec(rotvec, num=10):
    # v_2 = np.random.random((num, 3))
    v_2 = np.array([
        [1,0,0],
        [0,1,0],
        [0,0,1]])
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
        # return 

    def rjac(self, x):
        rot = R.from_rotvec(x)
        jacs = np.array([-rot.as_dcm().dot(hat(v)) for v in self.v2s])
        return jacs.reshape((len(jacs)*3, 3))

    def jac(self, x):
        rot = R.from_rotvec(x)
        jacs = np.vstack([-hat(rot.apply(v)) for v in self.v2s])
        return jacs

    def j_square(self, x):
        return self.f(x).transpose().dot(self.f(x))

    def d_cost_dx(self, x):
        return self.jac(x).transpose().dot(self.f(x)) + self.f(x).transpose().dot(self.jac(x))

    def cost(self, x):
        return self.f(x).transpose().dot(self.f(x))


def gauss_newton(problem, guess):
    x = guess
    for i in range(0, 10):
        j = problem.jac(x)
        b = -j.transpose().dot(problem.f(x))
        H = j.transpose().dot(j)
        update = np.linalg.solve(H, b)
        print('x: {}, ud: {}, dcost_dx:{}, cost: {}'.format(x, update, problem.d_cost_dx(x), problem.cost(x)))
        # x += update
        x = (R.from_rotvec(update) * R.from_rotvec(x)).as_rotvec()
        # x = R.from_dcm( R.from_rotvec(x).as_dcm().dot(R.from_rotvec(update).as_dcm()) ).as_rotvec()
    return x
        
def static_test():
    w2 = np.array([[1,0,0],[0,1,0],[0,0,1]])
    data_num = len(w2)
    rot = R.from_rotvec([1,1,1])
    w1 = rot.apply(w2)
    # print(w1)

    eps = 1e-5

    d_r = R.from_rotvec([
        [eps,0,0],
        [0,eps,0],
        [0,0,eps]])

    rot_inc = R.from_rotvec([[1 + eps,1,1],[1,1 + eps,1],[1,1,1 + eps]])

    print('d_residual_0_dr:')
    print(((d_r * rot).apply(w2[0]) - w1[0])/eps)
    print('d_residual_1_dr:')
    print(((d_r * rot).apply(w2[1]) - w1[1])/eps)
    print('d_residual_2_dr:')
    print(((d_r * rot).apply(w2[2]) - w1[2])/eps)

    print('residual(3x3):')
    residual = np.vstack([((rot).apply(w2[i]) - w1[i]) for i in range(data_num)])
    print(residual)

    print('residual_inc(3x3x3):')
    residual_inc = np.array(
        [np.vstack([ \
            ((rot_inc_).apply(w2[i]) - w1[i]) \
            for i in range(data_num)]) \
        for rot_inc_ in rot_inc])
    print(residual_inc)

    print('residual_ravel(9x1):')
    residual_ravel = residual.ravel()
    print(residual_ravel)

    print('d_residual_dr(9x3): ')
    d_residual_dr = np.vstack([((d_r * rot).apply(w2[i]) - w1[i]) for i in range(data_num)]) / eps
    print(d_residual_dr)

    print('j_residual(9x3): ')
    j_residual = np.vstack([hat(rot.apply(w2[i])) for i in range(data_num)])
    print(j_residual)

    print('numerical cost(1x1): ')
    numerical_cost = residual_ravel.transpose().dot(residual_ravel)
    print(numerical_cost)

    print('numerical cost inc(3x1): ')
    numerical_cost_incs = np.vstack([residual_inc_.ravel().dot(residual_inc_.ravel()) for residual_inc_ in residual_inc])
    print(numerical_cost_incs)

    print('numerical d_cost_dr(3x1): ')
    d_cost_dr = np.vstack([numerical_cost_inc - numerical_cost for numerical_cost_inc in numerical_cost_incs])/eps
    print(d_cost_dr)

    # j = - hat(rot.apply(w2))
    # print('j:')
    # print(j) 

def static_test2():
    w2 = np.array([[1,0,0],[0,1,0],[0,0,1]])
    real_vec = np.array([1,1,1])
    rot = R.from_rotvec(real_vec)
    w1 = rot.apply(w2)

    eps = 1e-5

    p = Problem(w1, w2)

    
    print(p.jac(real_vec))

    quit()

    # 
    real_square = p.j_square(real_vec)
    d_vec_dx = (p.j_square(real_vec + np.array([eps,0,0])) - real_square)/eps
    d_vec_dy = (p.j_square(real_vec + np.array([0,eps,0])) - real_square)/eps
    d_vec_dz = (p.j_square(real_vec + np.array([0,0,eps])) - real_square)/eps

    print(np.vstack((d_vec_dx, d_vec_dy, d_vec_dz))) 

if __name__ == "__main__":
    if len(sys.argv) > 1:
        static_test()
        quit()

    v1, v2 = GenerateDataByRotVec([1,1,1], num=3)

    p = Problem(v1, v2)
    
    guess = [2.+ (1e-5),3.,30]
    # print(p.jac(guess))
    
    # print(p.d_err(guess))
    print('residual')
    print(p.f(guess))
    print('j_residual')
    print(p.jac(guess))
    res = gauss_newton(p, guess)
    # print('compare: {}#{}'.format(R.from_rotvec(res).apply(v2[0]), v1[0]))
    
    # print(p.f(guess))
    # print(p.jac(guess).transpose().dot(p.f(guess)) + p.f(guess).transpose().dot(p.jac(guess)))
    # quit()
    res = least_squares(p.f, guess, jac=p.jac, method='lm',verbose=2, x_scale='jac')
    # res = least_squares(p.f, guess, method='lm',verbose=2)
    print(res.x)

    print('start')