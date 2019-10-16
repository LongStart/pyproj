import matplotlib.pyplot as plt
import numpy as np
from basis_function import *
import copy
from solver import GaussNewton
from scipy.optimize import least_squares
from solver import Problem
from bspline_utils import CreateUniformKnotVector


class BsplineFittingProblem(Problem):
    def __init__(self, degree, knot_vec, pts, pys):
        self.bsp = bspline(degree, knot_vec, np.ones((len(knot_vec) - degree - 1)))#copy.deepcopy(bsp)
        self.pts = pts[1:-1]
        self.pys = pys[1:-1]

    def local_bsp(self, x):
        temp = copy.deepcopy(self.bsp)
        temp.control_points =  self.state_vec_to_ctrl_points(x)
        return temp

    def residual(self, x):
        return self.local_bsp(x)(self.pts) - self.pys

    def state_vec_to_ctrl_points(self, x):
        return x

    def num_jac(self, x):
        eps = 1e-5
        d_res_d_x = []
        for i in range(len(x)):
            x_inc = np.array(x)
            x_inc[i] += eps
            # print(x_inc)
            print('{}#{}'.format(self.local_bsp(x_inc)(self.pts), self.local_bsp(x)(self.pts)))
            d_res_d_x.append((self.residual(x_inc) - self.residual(x))/eps)
        return np.vstack(d_res_d_x).transpose()

    def jac(self, x):
        return np.vstack([self.local_bsp(x).basis(i, self.pts) for i in range(len(x))]).transpose()

    @staticmethod
    def update(x, update):
        return x + update




if __name__ == "__main__":
    np.set_printoptions(precision=5, linewidth=np.inf)
    print ('66666!')
    sample_data = np.array([
        [1.,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16],
        [4.,5,6,3,2,4,6,6,4,5,6,4,4,5,3,4]])
    # knot_vec = CreateKnotVector1(3, [1.,3,5,6,8,9,10,14,15,16])
    degree = 3
    knot_vec = CreateUniformKnotVector(degree, 1, 16, 14)
    # print(knot_vec)
    # bsp = bspline(3, knot_vec, [1.]*(len(knot_vec) - 4))
    problem = BsplineFittingProblem(degree, knot_vec, sample_data[0], sample_data[1])

    # guess = [1.]*(len(knot_vec) - degree - 1)
    # print('num jac')
    # print(problem.num_jac(guess))
    # print('jac')
    # print(problem.jac(guess))
    # quit()
    result = problem.solve(problem.bsp.control_points, step=10)
    quit()
    problem.bsp.control_points = result
    # res = least_squares(problem.residual, guess, jac=problem.jac, method='lm',verbose=2)
    # result = res.x
    # print(res.x)

    plt.plot(sample_data[0], sample_data[1], 'o')
    resample_t = np.linspace(sample_data[0,0], sample_data[0, -1], 100)
    plt.plot(resample_t, problem.bsp(resample_t), '-')
    # x_fit, y_fit = problem.bsp.curve(100)
    # plt.plot(x_fit, y_fit, '-')
    plt.show()