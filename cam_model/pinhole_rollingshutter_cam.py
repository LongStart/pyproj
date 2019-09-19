import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
from projection import *
import cv2 as cv

class RollingShutterCamera():
    def __init__(self, resolution=[640, 480], model=RadTanPinhole(), rolling_time=30e-3):
        self.model = model
        self.resolution = resolution
        self.rolling_time = rolling_time

    def Project(self, points, cam_state):

        dt = self.rolling_time / self.resolution[1]
        states = cam_state.PredictNeighbors(dt, self.resolution[1] / 2)
        # print(states[:5])

        pts_uv = [PinholeCameraProjectPoint(points, state.ang_p, state.lin_p, self.model.intrinsic_mat) for state in states]
        # pts_uv = [cv.projectPoints(board.BodyFramePoints(), board.PoseInCameraFrame(cam_pose).ang_p, board.PoseInCameraFrame(cam_pose).lin_p, self.model.intrinsic_mat, self.model.distortion)[0] for cam_pose in states]
        
        roi_dict = {}
        for row_i in range(self.resolution[1]):
            frame_points_dict = GetPointInROI(pts_uv[row_i], [0., self.resolution[0]], [row_i, row_i + 1], pt_size=0.1)
            for idx, point in frame_points_dict.items():
                if roi_dict.has_key(idx):
                    roi_dict[idx].append(point)
                else:
                    roi_dict[idx] = [point]

        pts_uv_frame = []
        for idx, points in roi_dict.items():
            pts_uv_frame.append(sum(points) / len(points))
        return np.array(pts_uv_frame)
