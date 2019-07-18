import PlotCollection
import numpy as np
from add_3axis_figure import *
from scipy.spatial.transform import Rotation as R
from basis_function import basis
from basis_function import check_knot_vector

class BSplineSO3(object):
    def __init__(self, degree, knot_vector, control_points):
        assert(len(knot_vector) == degree + len(control_points) + 1)
        self.degree = degree
        self.knot_vector = check_knot_vector(knot_vector)
        self.control_points = control_points
        self.min_t = self.knot_vector[self.degree]
        self.max_t = self.knot_vector[-1-self.degree]

    def basis(self, i, t):
        return basis(self.degree, self.knot_vector, i, t)

    def __call__(self, t):
        # sum_y = np.zeros(len(t))
        cum_prod_y = R.from_quat([[0,0,0,1]]*len(t))
        for i in range(len(self.control_points)):
            y = [w * R.from_quat(self.control_points[i]).as_rotvec() for w in basis(self.degree, self.knot_vector, i, t)] 
            cum_prod_y *= R.from_rotvec(y)
        return cum_prod_y.as_quat()

if __name__ == "__main__":
    print('6666666')
    # data_num = 5
    # t = np.linspace(0, 10, data_num)
    
    # ctrl_t_xyzw = np.vstack([np.linspace(0, 10, data_num), R.random(data_num).as_quat().transpose()])

    deg = 2
    # knot_vector = np.array([0.,1,2,3,4,5,6,7,8,9,10,11,12,13,14])
    knot_vector = np.array([0.,1,2,3,4,5,6,7,8,9,10,11])
    control_points = R.from_rotvec([[0,0,i] for i in range(len(knot_vector) - deg - 1)]).as_quat()
    # control_points = R.random(len(knot_vector) - deg - 1).as_quat()
    # control_points = R.from_rotvec([[0,0,1]] * (len(knot_vector) - deg - 1)).as_quat()
    bsp = BSplineSO3(deg, knot_vector, control_points)

    sample_t = np.linspace(knot_vector[deg], knot_vector[-1-deg], 100)
    curve = bsp(sample_t)
    curve_t_xyzw = np.vstack([sample_t, curve.transpose()])
    
    # print(t_xyzw)
    ctrl_t_xyzw = np.vstack([knot_vector[deg / 2 : -1 - (deg/2)], control_points.transpose()])
    quat = {
        'q_raw': ctrl_t_xyzw,
        'q_bsp': curve_t_xyzw}
    plotter = PlotCollection.PlotCollection("Multiple Wave")
    add_naxis_figure(plotter, "orientation", quat, markersize=5, fmt='.')
    plotter.show()