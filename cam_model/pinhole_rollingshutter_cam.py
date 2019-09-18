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

class PoseState():
    def __init__(self, lin_p=np.zeros(3), lin_v=np.zeros(3), lin_a=np.zeros(3), ang_p=np.zeros(3), ang_v=np.zeros(3), ang_a=np.zeros(3))
        self.lin_p = lin_p
        self.lin_v = lin_v
        self.lin_a = lin_a
        self.ang_p = ang_p
        self.ang_v = ang_v
        self.ang_a = ang_a

    def Propagate(self, dt):
        lin_p = self.lin_p + self.lin_v * dt + 0.5 * self.lin_a * dt * dt
        lin_v = self.lin_v + self.lin_a * dt

        ang_p = (R.from_rotvec(0.5 * self.ang_a * dt * dt) * R.from_rotvec(self.ang_v * dt) * R.from_rotvec(self.ang_p)).as_rotvec()
        ang_v = self.ang_v + self.ang_a * dt
        return PoseState(lin_p=lin_p, lin_v=lin_v, lin_a=self.lin_a, ang_p=ang_p, ang_v=ang_v, ang_a=self.ang_a)

    def PropagateN(self, dt, len, half_step_start=True):
        states = []
        current_state = deepcopy(self)
        if half_step_start:
            current_state = self.Propagate(-.5 * dt)
        for i in range(len):
            current_state = current_state.Propagate(dt)
            states.append(current_state)
        return states

    def PredictNeighbors(self, dt, n_radius):
        states_up = self.PropagateN(dt, n_radius)
        states_down = self.PropagateN(-dt, n_radius).reverse()
        states = states_down + states_up
        return states


class RollingShutterCamera():
    def __init__(self, resolution=[640, 480], model=RadTanPinhole(), rolling_time=30e-3):
        self.model = model
        self.resolution = resolution
        self.rolling_time = rolling_time

    def Project(self, points, cam_state):
        # cam_state = np.zeros((2,3,3))
        # if cam_state_in.shape == (2,3,3):
        #     cam_state = cam_state_in
        # elif cam_state_in.shape == (2,3):
        #     cam_state[:, 0, :] = cam_state_in
        # else:
        #     raise TypeError("cam_state_in with shape: {} unavailable".format(cam_state_in.shape))

        dt = self.rolling_time / self.resolution[1]
        # poses_up = PoseList(cam_state, dt, self.resolution[1] / 2)
        # poses_down = PoseList(cam_state, -dt, self.resolution[1] / 2)
        # poses = np.vstack([np.flip(poses_down, axis=0), poses_up])
        states = cam_state.PredictNeighbors(dt, self.resolution[1] / 2)

        pts_uv = [PinholeCameraProjectPoint(points, state.ang_p, state.lin_p, self.model.intrinsic_mat) for state in states]

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
