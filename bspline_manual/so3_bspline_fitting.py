import matplotlib.pyplot as plt
import PlotCollection
import numpy as np
from so3_basis_function import *
import copy
from solver import GaussNewton
from solver import Problem
from scipy.optimize import least_squares
from bspline_utils import CreateUniformKnotVector
import time

class SO3BsplineFittingProblem(Problem):
    def __init__(self, bsp, ts, ys):
        self.bsp = copy.deepcopy(bsp)
        self.ts = ts[1:-1]
        self.ys = ys[1:-1]

    def local_bsp(self, x):
        temp = copy.deepcopy(self.bsp)
        temp.control_points =  self.state_vec_to_ctrl_points(x)
        return temp

    def residual(self, x):
        rot_ys = R.from_quat(self.ys)
        qs = R.from_rotvec(x.reshape(int(len(x)/3), 3)).as_quat()
        rot_sp = R.from_quat(self.local_bsp(qs)(self.ts))
        return  np.hstack((rot_ys.inv() * rot_sp).as_rotvec())

    def state_vec_to_ctrl_points(self, x):
        return x

    def num_jac(self, x):
        eps = 1e-5
        d_res_d_x = []
        rot_vecs = x.reshape(-1, 3)
        for rot_idx in range(len(rot_vecs)):
            for ax_idx in range(3):
                rot_vecs_inc = copy.deepcopy(rot_vecs)
                disturb = np.zeros(3)
                disturb[ax_idx] += eps
                rot_vecs_inc[rot_idx] =  (R.from_rotvec(disturb) * R.from_rotvec(rot_vecs_inc[rot_idx])).as_rotvec()
                d_res_d_x.append((self.residual(rot_vecs_inc.ravel()) - self.residual(x))/eps)
        return np.vstack(d_res_d_x).transpose()

    def jac(self, x):
        return self.num_jac(x)

    @staticmethod
    def update(x, update):
        rot_x = R.from_rotvec(x.reshape(-1, 3))
        rot_update = R.from_rotvec(update.reshape(-1, 3))
        return (rot_update * rot_x).as_rotvec().ravel()


if __name__ == "__main__":
    np.set_printoptions(precision=5, linewidth=np.inf)
    print ('66666!')
    # sample_data = np.array([
    #     [1.,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16],
    #     [4.,5,6,3,2,4,6,6,4,5,6,4,4,5,3,4]])
    sample_num = 15
    sample_data_t = np.array(range(sample_num))
    sample_data_p = R.from_rotvec([[0,0,.1 * i] for i in range(sample_num)]).as_quat()
    # knot_vec = CreateKnotVector1(3, [1.,2,3,5,7,9, 13, 15, 17, 18, 19])
    knot_vec = CreateUniformKnotVector(3, 1, 14, 13)
    print(knot_vec)
    bsp = BSplineSO3(3, knot_vec, [1.]*(len(knot_vec) - 4))
    problem = SO3BsplineFittingProblem(bsp, sample_data_t, sample_data_p)

    guess = np.array([0, 0, 1.]*(len(knot_vec) - 4))
    
    print(guess)
    # print('num jac')
    # print(problem.num_jac(guess))
    print('jac')
    # print(problem.jac(guess))
    # quit()
    # result = GaussNewton(problem, guess, step = 5)
    t0 = time.time()
    result = problem.solve(guess, step = 1, verbose=0)
    print('t0 = {}'.format(time.time() - t0))
    # res = least_squares(problem.residual, guess, jac=problem.jac, method='lm',verbose=2)
    # result = res.x
    print(result)
    t0 = time.time()
    problem.bsp.control_points = R.from_rotvec(result.reshape(-1,3)).as_quat()

    
    # spline_fit = BSplineSO3(3, knot_vec, R.from_rotvec(result.reshape(len(result)/3, 3)).as_quat())
    # spline_fit.curve(100)

    quat = {
        'q_raw': np.vstack([sample_data_t, sample_data_p.transpose()]),
        # 'q_sci': scipy_t_xyzw,
        'q_bsp': problem.bsp.curve(100)}
    plotter = PlotCollection.PlotCollection("Multiple Wave")
    add_naxis_figure(plotter, "orientation", quat, markersize=5, fmt='.-')
    print('t0 = {}'.format(time.time() - t0))
    plotter.show()