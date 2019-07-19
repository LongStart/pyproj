import matplotlib.pyplot as plt
import PlotCollection
import numpy as np
from so3_basis_function import *
import copy
from solver import GaussNewton
from scipy.optimize import least_squares

class SO3BsplineFittingProblem():
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
        qs = R.from_rotvec(x.reshape(len(x)/3, 3)).as_quat()
        rot_sp = R.from_quat(self.local_bsp(qs)(self.ts))
        return  np.hstack((rot_ys.inv() * rot_sp).as_rotvec())

    def state_vec_to_ctrl_points(self, x):
        return x

    def num_jac(self, x):
        eps = 1e-5
        d_res_d_x = []
        # bsp_input = R.from_rotvec(x.reshape(len(x)/3, 3)).as_quat()
        rot_vecs = x.reshape(len(x)/3, 3)
        # print(rot_vecs)
        for rot_idx in range(len(x)/3):
            for ax_idx in range(3):
                rot_vecs_inc = copy.deepcopy(rot_vecs)
                disturb = np.zeros(3)
                disturb[ax_idx] += eps
                rot_vecs_inc[rot_idx] =  (R.from_rotvec(disturb) * R.from_rotvec(rot_vecs_inc[rot_idx])).as_rotvec()
                d_res_d_x.append((self.residual(rot_vecs_inc.ravel()) - self.residual(x))/eps)
        return np.vstack(d_res_d_x).transpose()
                

        # for i in range(len(x)):
        #     x_inc = np.array(x)
        #     x_inc[i] += eps
        #     print('{}#{}'.format(self.local_bsp(x_inc)(self.ts), self.local_bsp(x)(self.ts)))
        #     d_res_d_x.append((self.residual(x_inc) - self.residual(x))/eps)
        # return np.vstack(d_res_d_x).transpose()

    def jac(self, x):
        # return np.vstack([self.local_bsp(x).basis(i, self.ts) for i in range(len(x))]).transpose()
        return self.num_jac(x)
    
    def d_cost_dx(self, x):
        return self.jac(x).transpose().dot(self.residual(x)) + self.residual(x).transpose().dot(self.jac(x))

    def cost(self, x):
        return self.residual(x).transpose().dot(self.residual(x))

    @staticmethod
    def update(x, update):
        rot_x = R.from_rotvec(x.reshape(len(x)/3, 3))
        rot_update = R.from_rotvec(update.reshape(len(x)/3, 3))
        return (rot_update * rot_x).as_rotvec().ravel()

def CreateKnotVector1(degree, time_stamps):
    front_len = (degree + 1) / 2
    back_len = front_len if degree % 2 == 1 else (front_len + 1)
    return np.hstack([[time_stamps[0]]* front_len, time_stamps, [time_stamps[-1]]* back_len])


if __name__ == "__main__":
    np.set_printoptions(precision=5, linewidth=np.inf)
    print ('66666!')
    # sample_data = np.array([
    #     [1.,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16],
    #     [4.,5,6,3,2,4,6,6,4,5,6,4,4,5,3,4]])
    sample_num = 20
    sample_data_t = np.array(range(sample_num))
    sample_data_p = R.from_rotvec([[0,0,.1 * i] for i in range(sample_num)]).as_quat()
    knot_vec = CreateKnotVector1(3, [1.,2,3,5,7,9, 13, 15, 17, 18, 19])
    print(knot_vec)
    bsp = BSplineSO3(3, knot_vec, [1.]*(len(knot_vec) - 4))
    problem = SO3BsplineFittingProblem(bsp, sample_data_t, sample_data_p)

    guess = np.array([0, 0, 1.]*(len(knot_vec) - 4))
    print(guess)
    # print('num jac')
    # print(problem.num_jac(guess))
    print('jac')
    print(problem.jac(guess))
    # quit()
    result = GaussNewton(problem, guess, step = 5)
    # res = least_squares(problem.residual, guess, jac=problem.jac, method='lm',verbose=2)
    # result = res.x
    print(result)

    
    spline_fit = BSplineSO3(3, knot_vec, R.from_rotvec(result.reshape(len(result)/3, 3)).as_quat())
    spline_fit.curve(100)

    quat = {
        'q_raw': np.vstack([sample_data_t, sample_data_p.transpose()]),
        # 'q_sci': scipy_t_xyzw,
        'q_bsp': spline_fit.curve(100)}
    plotter = PlotCollection.PlotCollection("Multiple Wave")
    add_naxis_figure(plotter, "orientation", quat, markersize=5, fmt='o-')
    plotter.show()