import unittest
# from core.dsp import *
import numpy as np
from scipy.spatial.transform import Rotation as R
# from core.lie_algebra import *
from space_signal import Trajectory3d


class TestTrajFunctions(unittest.TestCase):

    def test_trajectory_3d(self):
        raw_data = np.array([
            [1,2,3,4],
            [2,4,6,8],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [1,1,1,1]])

        traj = Trajectory3d(raw_data)
        np.testing.assert_array_equal(traj.t(), raw_data[0])
        np.testing.assert_array_equal(traj.xyz(), raw_data[1:4])
        np.testing.assert_array_equal(traj.xyzw(), raw_data[4:])
        self.assertRaises(AssertionError, Trajectory3d, [[1,2,3]])

        update_t = [2,3,4,5]
        traj.t(update_t)
        np.testing.assert_array_equal(traj.t(), update_t)

        print(traj.t_xyz_xyzw())

if __name__ == '__main__':
    unittest.main()

    