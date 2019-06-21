import unittest
from dsp import *
import numpy as np
from scipy.spatial.transform import Rotation as R
from evo.core import lie_algebra

class TestDspFunctions(unittest.TestCase):

    def test_angle_velocity_from_dcm(self):
        len = 7
        rot_axis = np.array([0,0,1])
        v = 0.1
        t_in = np.array([v*t for t in range(len)])
        r_in = R.from_rotvec([v*rot_axis*t for t in range(len)])
        omegas = DerivativeRot(t_in, r_in)
        result = np.array([v*rot_axis]*(len-2))

        self.assertTrue(omegas[0].all() == t_in[1:-1].all())
        self.assertTrue(omegas[1].all() == result.all())

    # def test_rotation_derivative(self):
    #     len = 7
    #     rot_axis = np.array([0,0,1])
    #     t_in = np.array([0.1*t for t in range(len)])
    #     r_in = R.from_rotvec([0.1*rot_axis*t for t in range(len)])
    #     dr_dt = DerivativeRot(t_in, r_in)
    #     print(dr_dt)
    #     omega_hat = np.array([[0, -1, 0],[1, 0, 0],[0, 0, 0]])
    #     # result = np.array([r*omega_hat])
    #     self.assertTrue(dr_dt[0].all() == t_in[1:-1].all())
    #     # self.assertListEqual(list(dr_dt[0]), list(t_in[1:-1]))


if __name__ == '__main__':
    unittest.main()

    