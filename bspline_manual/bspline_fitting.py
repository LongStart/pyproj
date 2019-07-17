import matplotlib.pyplot as plt
import numpy as np
from basis_function import *
import copy
from solver import GaussNewton

def CreateKnotVector(degree, time_stamps):
    front_len = (degree + 1) / 2 + 1
    back_len = front_len if degree % 2 == 1 else (front_len + 1)
    return np.hstack([[time_stamps[1]]* front_len, time_stamps[1:-1], [time_stamps[-2]]* back_len])

def CreateKnotVector1(degree, time_stamps):
    front_len = (degree + 1) / 2
    back_len = front_len if degree % 2 == 1 else (front_len + 1)
    return np.hstack([[time_stamps[0]]* front_len, time_stamps, [time_stamps[-1]]* back_len])

class BsplineFittingProblem():
    def __init__(self, bsp, pxs, pys):
        self.bsp = copy.deepcopy(bsp)
        self.pxs = pxs
        self.pys = pys

    def temp_bsp(self, control_points):
        temp = copy.deepcopy(self.bsp)
        temp.control_points = control_points
        return temp

    def residual(self, control_points):
        # return self.temp_bsp(control_points)
        return self.temp_bsp(control_points)(self.pxs) - self.pys

    def num_jac(self, control_points):
        eps = 1e-8
        d_res_d_x = []
        for i in range(len(control_points)):
            control_points_inc = np.array(control_points)
            control_points_inc[i] += eps
            # print('{}#{}'.format((self.temp_bsp(control_points_inc)(self.pxs) - self.temp_bsp(control_points)(self.pxs))/eps, self.temp_bsp(control_points).basis(i, self.pxs)))
            d_res_d_x.append((self.residual(control_points_inc) - self.residual(control_points))/eps)
        return np.vstack(d_res_d_x)

    def jac(self, control_points):
        return np.vstack([self.temp_bsp(control_points).basis(i, self.pxs) for i in range(len(control_points))])
    
    def d_cost_dx(self, x):
        return self.jac(x).transpose().dot(self.f(x)) + self.f(x).transpose().dot(self.jac(x))

    def cost(self, x):
        return self.f(x).transpose().dot(self.f(x))

    @staticmethod
    def update(control_points, update):
        return control_points + update




if __name__ == "__main__":
    np.set_printoptions(precision=4, linewidth=np.inf)
    print ('66666!')
    sample_data = np.array([[1.,2,3,4,5,6,7],[4.,5,6,3,2,4,6]])
    knot_vec = CreateKnotVector(3, sample_data[0])
    print(knot_vec)
    bsp = bspline(3, knot_vec, sample_data[1])
    problem = BsplineFittingProblem(bsp, sample_data[0], sample_data[1])

    guess = [1,0,0,0,0,0,0]
    print(problem.num_jac(guess))
    print(problem.jac(guess))
    quit()
    result = GaussNewton(problem, guess, step = 10)
    

    # plt.plot(sample_data[0], sample_data[1], 'o')
    # plt.show()