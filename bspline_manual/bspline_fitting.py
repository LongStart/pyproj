import matplotlib.pyplot as plt
import numpy as np
from basis_function import *
import copy
from solver import GaussNewton
from scipy.optimize import least_squares

def CreateKnotVector(degree, time_stamps):
    front_len = (degree + 1) / 2 + 1
    back_len = front_len if degree % 2 == 1 else (front_len + 1)
    return np.hstack([[time_stamps[1]]* front_len, time_stamps[1:-1], [time_stamps[-2]]* back_len])

def CreateKnotVector1(degree, time_stamps):
    front_len = (degree + 1) / 2
    back_len = front_len if degree % 2 == 1 else (front_len + 1)
    return np.hstack([[time_stamps[0]]* front_len, time_stamps, [time_stamps[-1]]* back_len])

class BsplineFittingProblem():
    def __init__(self, bsp, pxs, pys, cp_front, cp_back):
        self.bsp = copy.deepcopy(bsp)
        self.pxs = pxs[1:-1]
        self.pys = pys[1:-1]
        self.cp_front = cp_front
        self.cp_back = cp_back

    def local_bsp(self, x):
        temp = copy.deepcopy(self.bsp)
        temp.control_points =  np.hstack([self.cp_front, x, self.cp_back]) 
        return temp

    def residual(self, x):
        return self.local_bsp(x)(self.pxs) - self.pys

    def num_jac(self, x):
        eps = 1e-5
        d_res_d_x = []
        for i in range(len(x)):
            x_inc = np.array(x)
            x_inc[i] += eps
            # print(x_inc)
            print('{}#{}'.format(self.local_bsp(x_inc)(self.pxs), self.local_bsp(x)(self.pxs)))
            d_res_d_x.append((self.residual(x_inc) - self.residual(x))/eps)
        return np.vstack(d_res_d_x).transpose()

    def jac(self, x):
        return np.vstack([self.local_bsp(x).basis(i+1, self.pxs) for i in range(len(x))]).transpose()
    
    def d_cost_dx(self, x):
        return self.jac(x).transpose().dot(self.residual(x)) + self.residual(x).transpose().dot(self.jac(x))

    def cost(self, x):
        return self.residual(x).transpose().dot(self.residual(x))

    @staticmethod
    def update(x, update):
        return x + update




if __name__ == "__main__":
    np.set_printoptions(precision=5, linewidth=np.inf)
    print ('66666!')
    sample_data = np.array([[1.,2,3,4,5,6,7],[4.,5,6,3,2,4,6]])
    knot_vec = CreateKnotVector1(3, sample_data[0])
    print(knot_vec)
    bsp = bspline(3, knot_vec, sample_data[1])
    problem = BsplineFittingProblem(bsp, sample_data[0], sample_data[1], 1, 1)

    guess = [1.,1,1,1,1]
    print('num jac')
    print(problem.num_jac(guess))
    print('jac')
    print(problem.jac(guess))
    # quit()
    # result = GaussNewton(problem, guess, step = 10)
    res = least_squares(problem.residual, guess, jac=problem.jac, method='lm',verbose=2)
    result = res.x
    # print(res.x)

    
    spline_fit = bspline(3, knot_vec, np.hstack([1, result, 1]))

    plt.plot(sample_data[0], sample_data[1], 'o')
    x_fit, y_fit = spline_fit.curve()
    plt.plot(x_fit, y_fit, '-')
    plt.show()