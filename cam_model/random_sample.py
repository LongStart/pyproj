import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

from projection import *
from random_pose_spline import *
from pinhole_rollingshutter_cam import RollingShutterCamera

import cv2 as cv
import sys

class CalibrationSampler(object):
    def __init__(self, sample_num=10, ctrl_point_num=20, time=60, camera=PinholeCamera()):
        self.board = CalibrationBoard(orientation=[math.pi / 2, 0, 0], size_w_h=[6,4])
        self.trajectory = TargetOrientationPoseSpline(target_point=self.board.center_global, ctrl_point_num=ctrl_point_num, time=time, random_range=[[-.6,-.7, -.6],[.6,-.8, .6]])
        # self.trajectory = TargetOrientationPoseSpline(target_point=self.board.center_global, ctrl_point_num=ctrl_point_num, time=time, random_range=[[0,-.8, 0],[0.,-.8, 0]])
        self.camera = camera
        self.cam_states = np.zeros((sample_num, 2))
        self.tf_board_to_cam = np.zeros((sample_num, 2))
        self.UpdateSample(sample_num)

    @property
    def sample_num(self):
        return len(self.cam_states)

    def UpdateSample(self, sample_num=10):
        sample_t = np.linspace(0, self.trajectory.time, sample_num)
        self.cam_states = []
        for t in sample_t:
            self.cam_states.append([R.from_dcm(self.trajectory.bsp.orientation(t)).as_rotvec(), self.trajectory.bsp.position(t)])
        self.cam_states = np.array(self.cam_states)
        self.tf_board_to_cam = np.zeros(self.cam_states.shape)

        self.tf_board_to_cam[:,0] = (R.from_rotvec(-self.cam_states[:,0]) * self.board.orientation).as_rotvec()
        self.tf_board_to_cam[:,1] = R.from_rotvec(-self.cam_states[:,0]).apply(self.board.position - self.cam_states[:,1])

    def ProjectedPoints(self):
        return np.array([self.camera.Project(self.board.Points(), cam_pose) for cam_pose in self.cam_states])

    def BodyFramePoints(self):
        return np.array([self.board.BodyFramePoints()] * self.sample_num)

    def State(self, t):
        state = np.zeros((2,3,3))

        state[0, 0, :] = self.trajectory.bsp.position(t)
        state[0, 1, :] = self.trajectory.bsp.linearVelocity(t)
        state[0, 2, :] = self.trajectory.bsp.linearVelocity(t)
        state[0, 0, :] = R.from_dcm(self.trajectory.bsp.orientation(t)).as_rotvec()
        state[0, 1, :] = self.trajectory.bsp.angularVelocity(t)
        state[0, 2, :] = [0., 0., 0.]
        return  state

if __name__ == "__main__":
    sampler = CalibrationSampler(camera=RollingShutterCamera())
    sampler.camera.model.distortion = np.array([0.2, -0.1, 0, 0, 2])

    #opencv calibration
    if 0:
        sampler.UpdateSample(20)
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
            sampler.BodyFramePoints().astype('float32'),
            sampler.ProjectedPoints().astype('float32'),
            tuple(sampler.camera.resolution[::-1]), None, None)
        print(dist)

    sampler.UpdateSample(10)
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

