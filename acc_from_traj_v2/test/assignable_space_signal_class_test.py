import unittest
# from core.dsp import *
import numpy as np
from scipy.spatial.transform import Rotation as R
# from core.lie_algebra import *
from core.assignable_space_signal import Signal3d
from core.assignable_space_signal import Signal1d
from core.assignable_space_signal import Trajectory3d


class TestTrajFunctions(unittest.TestCase):

    def test_signal_3d(self):
        raw_data = np.array([
            [1,2,3,4],
            [2,4,6,8],
            [0,0,0,0],
            [0,0,0,0]])

        sig = Signal3d(raw_data)

        np.testing.assert_array_equal(sig.t, raw_data[0])
        np.testing.assert_array_equal(sig.xyz, raw_data[1:])
        sig.t = raw_data[1]
        np.testing.assert_array_equal(sig.t, raw_data[1])

        sig = Signal3d.from_t_xyz(raw_data[0])
        sig = Signal3d.from_t_xyz(raw_data[0])

    def test_signal_1d(self):
        t = np.array([1,2,3,4])

        x = np.array([2,4,6,8])

        sig = Signal1d.from_t_x(t)
        sig = Signal1d.from_t_x(t, x)
        sig = Signal1d(np.vstack((t,x)))


    def test_traj_3d(self):
        t = np.array([1,2,3,4])

        xyz = np.array([
            [2,4,6,8],
            [0,0,0,0],
            [0,0,0,0]])

        xyzw = np.array([
            [0,0,0,0],
            [0,0,0,0],
            [0,0,0,0],
            [1,1,1,1]])

        traj = Trajectory3d.from_t_xyz_xyzw(t)
        traj = Trajectory3d(np.vstack((t, xyz, xyzw)))
        

        np.testing.assert_array_equal(traj.t, t)
        np.testing.assert_array_equal(traj.xyz, xyz)
        traj.t = xyz[0]
        np.testing.assert_array_equal(traj.t, xyz[0])
        a = traj.t_xyz
        traj.t_xyz = np.vstack((t, xyz))

if __name__ == '__main__':
    unittest.main()

    