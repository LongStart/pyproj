import unittest
import numpy as np
from pinhole_rollingshutter_cam import *
import matplotlib.pyplot as plt

class TestStatePropagationFunctions(unittest.TestCase):
    def test_state_propagate(self):
        init_state = np.zeros((2,3,3))
        init_state[0,1,0] = 0.1 # vx = 0.1
        height = 10
        poses = PoseList(init_state, 1e-3/height, height / 2)
        # print(poses)

    def test_project(self):
        np.set_printoptions(precision=10, linewidth=np.inf)
        board = CalibrationBoard(position=[0., 0., 0], orientation=[math.pi/2,0.,0.], size_w_h=[8,6])
        cam = RollingShutterCamera(model=RadTanPinhole(distortion=[0.2, -0.1, 0, 0, 2]))
        cam_state = np.zeros((2,3,3))
        cam_state[:,0,:] = np.array([[-math.pi/2,0.,0.],[0.05, -0.8, 0.05]])
        # cam_state[1,1,0] = 0.1 # vx = 0.1
        cam_state[0,1,1] = 10 # angle_vx = 0.1
        points = cam.Project(board.Points(), cam_state)
        plt.plot(points.T[0], points.T[1], '.')
        plt.show()


if __name__ == '__main__':
    unittest.main()