import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
from projection import *

def PropagateState(state, dt):
    [angle_p, angle_v, angle_a], [p, v, a] = deepcopy(state)

    p += v * dt + 0.5 * a * dt * dt
    v += a * dt

    angle_p = (R.from_rotvec(0.5 * angle_a * dt * dt) * R.from_rotvec(angle_v * dt) * R.from_rotvec(angle_p)).as_rotvec()
    angle_v += angle_a * dt

    return np.array([[angle_p, angle_v, angle_a], [p, v, a]])

def PoseList(init_state, dt, len, half_step_start=True):
    poses = []
    current_state = init_state
    if half_step_start:
        current_state = PropagateState(init_state, -.5 * dt)
    for i in range(len):
        current_state = PropagateState(current_state, dt)
        poses.append(current_state[:,0,:])
    return np.array(poses)

class RollingShutterCamera():
    def __init__(self, resolution=[640, 480], model=RadTanPinhole(), rolling_time=30e-3):
        self.model = model
        self.resolution = resolution
        self.rolling_time = rolling_time

    def Project(self, points, cam_state):
        poses = PoseList(cam_state, self.rolling_time / self.resolution[1], self.resolution[1] / 2)
        pts_uv = [PinholeCameraProjectPoint(points, pose_rt[0], pose_rt[1], self.model.intrinsic_mat) for pose_rt in poses]
        return np.array(pts_uv)
