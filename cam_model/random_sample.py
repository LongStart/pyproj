import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from copy import deepcopy

from projection import *
from random_pose_spline import *
from pinhole_rollingshutter_cam import RollingShutterCamera

import cv2 as cv
import sys

class PoseState():
    def __init__(self, lin_p=np.zeros(3), lin_v=np.zeros(3), lin_a=np.zeros(3), ang_p=np.zeros(3), ang_v=np.zeros(3), ang_a=np.zeros(3)):
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
        states_down = self.PropagateN(-dt, n_radius)
        states_down.reverse()
        states = states_down + states_up
        return states

class CalibrationSampler(object):
    def __init__(self, sample_num=10, ctrl_point_num=20, time=60, camera=PinholeCamera()):
        self.board = CalibrationBoard(orientation=[math.pi / 2, 0, 0], size_w_h=[6,4])
        self.trajectory = TargetOrientationPoseSpline(target_point=self.board.center_global, ctrl_point_num=ctrl_point_num, time=time, random_range=[[-.6,-.7, -.6],[.6,-.8, .6]])
        # self.trajectory = TargetOrientationPoseSpline(target_point=self.board.center_global, ctrl_point_num=ctrl_point_num, time=time, random_range=[[0,-.8, 0],[0.,-.8, 0]])
        self.camera = camera
        self.cam_states = np.zeros((sample_num, 2))
        self.tf_board_to_cam = Pose()
        self.UpdateSample(sample_num)

    @property
    def sample_num(self):
        return len(self.cam_states)

    def UpdateSample(self, sample_num=10):
        sample_t = np.linspace(0, self.trajectory.time, sample_num)
        self.cam_states = [self.State(t) for t in sample_t]
        self.tf_board_to_cam = [self.board.PoseInCameraFrame(cam_state) for cam_state in self.cam_states]
        # for t in sample_t:
        #     self.cam_states.append([R.from_dcm(self.trajectory.bsp.orientation(t)).as_rotvec(), self.trajectory.bsp.position(t)])
        # self.cam_states = np.array(self.cam_states)
        # self.tf_board_to_cam = np.zeros(self.cam_states.shape)
        
        # self.tf_board_to_cam[:,0] = (R.from_rotvec(-self.cam_states[:,0]) * self.board.orientation).as_rotvec()
        # self.tf_board_to_cam[:,1] = R.from_rotvec(-self.cam_states[:,0]).apply(self.board.position - self.cam_states[:,1])

    def ProjectedPoints(self):
        return np.array([self.camera.Project(self.board.Points(), cam_pose) for cam_pose in self.cam_states])

    def BodyFramePoints(self):
        return np.array([self.board.BodyFramePoints()] * self.sample_num)

    def State(self, t):
        state = PoseState()
        state.lin_p = self.trajectory.bsp.position(t)
        state.lin_v = self.trajectory.bsp.linearVelocity(t)
        state.lin_a = self.trajectory.bsp.linearAcceleration(t)
        state.ang_p = R.from_dcm(self.trajectory.bsp.orientation(t)).as_rotvec()
        state.ang_v = self.trajectory.bsp.angularVelocity(t)
        return state

    # def BoardPosesInCameraFrame(self, cam_state):
    #     pose = Pose()
    #     pose.ang_p = (R.from_rotvec(-cam_state.ang_p) * self.board.orientation).as_rotvec()
    #     pose.lin_p = R.from_rotvec(-cam_state.ang_p).apply(self.board.position - cam_state.lin_p)
    #     return pose

if __name__ == "__main__":
    sampler = CalibrationSampler(camera=RollingShutterCamera())
    # sampler = CalibrationSampler(camera=PinholeCamera())
    sampler.camera.model.distortion = np.array([0.2, -0.1, 0, 0, 2])

    #opencv calibration
    if 0:
        sampler.UpdateSample(20)
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            sampler.BodyFramePoints().astype('float32'),
            sampler.ProjectedPoints().astype('float32'),
            tuple(sampler.camera.resolution[::-1]), None, None)
        print(dist)

    sampler.UpdateSample(60)
    board_points = sampler.ProjectedPoints()

    #animation
    if 1:
        for sample in board_points:
            fig_uv = plt.clf()
            plt.axis([0, sampler.camera.resolution[0], sampler.camera.resolution[1], 0])
            plt.plot(sample.T[0], sample.T[1], "*")
            plt.plot(sampler.camera.resolution[0]/2, sampler.camera.resolution[1]/2, "+", markersize=10)
            plt.grid(1)
            plt.pause(1e-6)

    #3d plot
    if 1:
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_aspect('equal')
        ax.scatter(*sampler.board.Points().T)
        ax.scatter([0], [0], [0], color='g', marker='+')
        ax.plot(*sampler.trajectory.VizPositionCurve(), color='r')
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.set_zlim(-3, 3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.show()

