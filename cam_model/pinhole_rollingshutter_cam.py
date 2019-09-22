import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
from projection import *
import cv2 as cv
from rolling_shutter import *

class RollingShutterCamera():
    def __init__(self, resolution=[640, 480], model=RadTanPinhole(), rolling_time=30e-3):
        self.model = model
        self.resolution = resolution
        self.rolling_time = rolling_time

    def Project(self, points, cam_state):

        dt = self.rolling_time / self.resolution[1]
        states = cam_state.PredictNeighbors(dt, self.resolution[1] / 2)
        project_shape = (len(points),2)
        # print(states[:5])

        # pts_uv = [PinholeCameraProjectPoint(points, state.ang_p, state.lin_p, self.model.intrinsic_mat) for state in states]
        pts_uv = [np.resize(cv.projectPoints(points, cam_pose.inv().ang_p, cam_pose.inv().lin_p, self.model.intrinsic_mat, self.model.distortion)[0], project_shape) for cam_pose in states]        
        print('project done')
        return RollingShutterFuse(pts_uv, self.resolution[0])

