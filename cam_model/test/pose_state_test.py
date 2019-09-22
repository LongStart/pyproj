import unittest
import numpy as np
import matplotlib.pyplot as plt
from rolling_shutter import *

class TestPoseState(unittest.TestCase):
    def test_pose_state(self):
        s = PoseState()
        s.lin_a = [1,2,3.2333333333]
        ns = s.PredictNeighbors(1e-3, 10)
        # print(ns)

    def test_inv(self):
        s = PoseState()
        s.lin_p = [1,2,3]
        print(s.inv())
    
    def test_rs_fuse(self):
        # mats = VectorMatrix() 
        # print(mats[0])
        # mats.append(np.matrix([[1,2],[2,3]]))
        mats = np.array([[[1,2],[3,4]]])
        RollingShutterFuse(mats)



if __name__ == '__main__':
    unittest.main()