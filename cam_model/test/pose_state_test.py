import unittest
import numpy as np
import matplotlib.pyplot as plt
from pose_state import PoseState

class TestPoseState(unittest.TestCase):
    def test_pose_state(self):
        s = PoseState()
        s.lin_a = [1,2,3.2333333333]
        ns = s.PredictNeighbors(1e-3, 10)
        print(ns)



if __name__ == '__main__':
    unittest.main()