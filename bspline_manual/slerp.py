import PlotCollection
import numpy as np
from add_3axis_figure import *
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

def slerp(q0, q1, t):
    rot_vec0 = R.from_quat(q0).as_rotvec()
    rot_vec1 = R.from_quat(q1).as_rotvec()

    item_0 = R.from_rotvec([(1. - tt)*rot_vec0 for tt in t])
    item_1 = R.from_rotvec([tt * rot_vec1 for tt in t])
    slerp_rot = item_0 * item_1
    return slerp_rot.as_quat()

def slerp1(q0, q1, t):
    rot_0 = R.from_quat(q0)
    d_rot = (R.from_quat(q0).inv() * R.from_quat(q1) ).as_rotvec()

    d_rots = R.from_rotvec([d_rot * tt for tt in t])
    slerp_rot = rot_0 * d_rots
    return slerp_rot.as_quat()

if __name__ == "__main__":
    sample_t = np.linspace(0.01, 0.99, 20)
    ctrl_t = np.array([0,1])
    rots = R.from_rotvec([[0,0,1], [1,0,0]])

    
    slerp_f = Slerp(ctrl_t, rots)
    scipy_slerp = slerp_f(sample_t)

    ctrl_t_xyzw = np.vstack([ctrl_t, rots.as_quat().transpose()])

    qs = slerp1(rots[0].as_quat(), rots[1].as_quat(), sample_t)
    curve_t_xyzw = np.vstack([sample_t, qs.transpose()])
    print(curve_t_xyzw)
    scipy_t_xyzw = np.vstack([sample_t, scipy_slerp.as_quat().transpose()])
    quat = {
        'q_raw': ctrl_t_xyzw,
        # 'q_sci': scipy_t_xyzw,
        'q_bsp': curve_t_xyzw}
    plotter = PlotCollection.PlotCollection("Multiple Wave")
    add_naxis_figure(plotter, "orientation", quat, markersize=5, fmt='-.')
    plotter.show()