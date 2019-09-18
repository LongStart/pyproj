import unittest
import numpy as np
from pinhole_rollingshutter_cam import *
import matplotlib.pyplot as plt
from random_sample import *

class TestStatePropagationFunctions(unittest.TestCase):
    def test_state_propagate(self):
        init_state = np.zeros((2,3,3))
        init_state[0,1,0] = 0.1 # vx = 0.1
        height = 10
        poses = PoseList(init_state, 1e-3/height, height / 2)
        # print(poses)

    def test_project(self):
        # return
        np.set_printoptions(precision=10, linewidth=np.inf)
        board = CalibrationBoard(position=[0., 0., 0], orientation=[math.pi/2,0.,0.], size_w_h=[16,9], spacing=0.02)
        # cam = RollingShutterCamera(model=RadTanPinhole(distortion=[0.2, -0.1, 0, 0, 2]))
        cam = RollingShutterCamera(model=RadTanPinhole(distortion=[.2, -0.1, 0, 0, 2]), rolling_time=30e-3)
        
        cam_state = PoseState(lin_p=np.array([.16, -0.8, 0.09]), ang_p=np.array([-math.pi/2,0.,0.]))
        # cam_state.lin_v[0] = 0.1
        cam_state.ang_v[0] = 0.
        cam_state.ang_v[1] = 20
        # cam_state[0,1,1] = 20 # angle_vx = 0.1
        points = cam.Project(board.Points(), cam_state)
        plt.plot(points.T[0], points.T[1], '.')
        plt.axis([0, cam.resolution[0], cam.resolution[1], 0])
        plt.show()


if __name__ == '__main__':
    unittest.main()