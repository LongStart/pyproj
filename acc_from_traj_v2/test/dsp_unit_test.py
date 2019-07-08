import unittest
from dsp import *
import numpy as np
from scipy.spatial.transform import Rotation as R
from lie_algebra import *
from trajectory_signal import Trajectory3d


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

    def test_lie_hat(self):
        v = np.array([1,2,3])
        result = np.array([[0, -3, 2],
                            [3, 0, -1],
                            [-2, 1, 0]])
        np.testing.assert_array_equal(hat(v), result)

    def test_lie_hat(self):
        v = np.array([[1,2,3],[1,2,3]])
        result = np.array([[[0, -3, 2],
                            [3, 0, -1],
                            [-2, 1, 0]]]*2)
        np.testing.assert_array_equal(hat(v), result)


if __name__ == '__main__':
    unittest.main()

    