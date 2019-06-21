import unittest
from dsp import *
import numpy as np
from scipy.spatial.transform import Rotation as R

class TestDspFunctions(unittest.TestCase):

    def test_angle_velocity_from_dcm(self):
        len = 7
        rot_axis = np.array([0,0,1])
        v = 0.1
        t_in = np.array([v*t for t in range(len)])
        r_in = R.from_rotvec([v*rot_axis*t for t in range(len)])
        q_in = r_in.as_quat().transpose()
        omegas = AngleRate(t_in, q_in)
        print(omegas)


if __name__ == '__main__':
    unittest.main()

    