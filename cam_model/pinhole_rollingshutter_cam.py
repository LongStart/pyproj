import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
from projection import *
import cv2 as cv
from rolling_shutter import *

def RollingShutterProject(points, cam_state, resolution, rolling_time, intrinsic_mat, distortion):
    dt = rolling_time / resolution[1]
    states = cam_state.PredictNeighbors(dt, resolution[1] / 2)
    project_shape = (len(points),2)
    pts_uv = [np.resize(cv.projectPoints(points, cam_pose.inv().ang_p, cam_pose.inv().lin_p, intrinsic_mat, distortion)[0], project_shape) for cam_pose in states]
    return RollingShutterFuse(pts_uv, resolution[0])


class RollingShutterCamera():
    def __init__(self, resolution=[640, 480], model=RadTanPinhole(), rolling_time=30e-3):
        self.model = model
        self.resolution = resolution
        self.rolling_time = rolling_time

    def Project(self, points, cam_state):

        dt = self.rolling_time / self.resolution[1]
        states = cam_state.PredictNeighbors(dt, self.resolution[1] / 2)
        project_shape = (len(points),2)
        # print(points.shape)
        if 0:
            pts_uv = [np.resize(cv.projectPoints(points, cam_pose.inv().ang_p, cam_pose.inv().lin_p, self.model.intrinsic_mat, self.model.distortion)[0], project_shape) for cam_pose in states]
            return RollingShutterFuse(pts_uv, self.resolution[0])

        else:
            # pts_uv = [np.resize(cv.projectPoints(points, cam_pose.inv().ang_p, cam_pose.inv().lin_p, self.model.intrinsic_mat, self.model.distortion)[0], project_shape) for cam_pose in states]
            pts_uv = [PinholeProject(points, cam_pose.inv().ang_p, cam_pose.inv().lin_p, self.model.intrinsic_mat, self.model.distortion) for cam_pose in states]
            # pts_uv = [PinholeCameraProjectPoint(points, state.ang_p, state.lin_p, self.model.intrinsic_mat) for state in states]
            # print("ddd")
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

