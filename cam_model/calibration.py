import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
from copy import deepcopy

from projection import *
from random_pose_spline import *
from random_sample import CalibrationSampler

import cv2 as cv
from solver import Problem
import matplotlib.cm as cm

class StateVector():
    def __init__(self, frame_num=0):
        self.frame_pose = np.zeros((frame_num, 2, 3))
        # self.intrinsic_ = np.zeros(4)
        self.set_intrinsic(np.zeros(4))
        self.distortion = np.zeros(5)

    def __eq__(self, other):
        return (self.frame_pose == other.frame_pose).all() \
        and (self.intrinsic_ == other.intrinsic_).all() \
        and (self.distortion == other.distortion).all()

    def __ne__(self, other):
        return not self == other

    def __str__(self):
        return "intrinsic: {}, dist: {},\nrt: {}".format(self.intrinsic_, self.distortion, self.frame_pose)

    @staticmethod
    def update(this, other):
        result = deepcopy(this)
        result.intrinsic_ += other.intrinsic_
        result.distortion += other.distortion
        
        result.frame_pose[:, 1] += other.frame_pose[:, 1]
        result.frame_pose[:, 0] = (R.from_rotvec(other.frame_pose[:, 0]) * R.from_rotvec(this.frame_pose[:, 0])).as_rotvec()
        # print(other.intrinsic_)
        # print("update: {}".format(other))
        return result

    def set_intrinsic(self, val):
        assert(val.shape == (4,))
        # print("set intrinsic: {}".format(val))
        self.intrinsic_ = val
    
    def get_intrinsic(self):
        mat = np.identity(3)
        mat[0,0] = self.intrinsic_[0]
        mat[1,1] = self.intrinsic_[1]
        mat[0,2] = self.intrinsic_[2]
        mat[1,2] = self.intrinsic_[3]
        return mat.astype('float32')

    # intrinsic = property(get_intrinsic, set_intrinsic)

    def rotation(self, frame_idx):
        return self.frame_pose[frame_idx, 0]

    def translation(self, frame_idx):
        return self.frame_pose[frame_idx, 1]

    def set_rotation(self, frame_idx, val):
        assert(val.shape == (3,))
        self.frame_pose[frame_idx, 0] = val

    def set_translation(self, frame_idx, val):
        assert(val.shape == (3,))
        self.frame_pose[frame_idx, 1] = val

    # @property
    def get_data(self):
        return np.hstack([self.intrinsic_, self.distortion, self.frame_pose.ravel()])

    def set_data(self, val):
        self.intrinsic_ = val[:4]
        self.distortion = val[4:9]
        self.frame_pose = np.reshape(val[9:], self.frame_pose.shape)
    
    data = property(get_data, set_data)

class PinholeCalibrationProblem(Problem):
    def __init__(self, pts_3d, pts_2d):
        assert(pts_3d.shape[:2] == pts_2d.shape[:2])
        self.pts_3d = pts_3d
        self.pts_2d = pts_2d
        self.frame_num, self.point_num = self.pts_2d.shape[:2]
        self.last_x = StateVector(self.frame_num)
        self.last_jac = np.array([0.] * self.point_num * 2)
        
    def frameResidual(self, x, idx):
        # print(x.get_intrinsic())
        # cv_rotation = R.from_rotvec([0,0,math.pi]) * R.from_rotvec(x.rotation(idx))
        # print("rot: {}".format(x.rotation(idx)))
        # print("trans: {}".format(x.translation(idx)))
        # print("dist: {}".format(x.distortion))
        # print("intrin: {}".format(x.get_intrinsic()))
        # print("self.pts_3d[idx]: {}".format(self.pts_3d[idx]))
        project_2d, cv_jac = cv.projectPoints(self.pts_3d[idx], x.rotation(idx), x.translation(idx), x.get_intrinsic(), x.distortion)
        frame_residual = project_2d.ravel() - self.pts_2d[idx].ravel()
        # print("projected: ")
        # print(project_2d)
        # print("stored: ")
        # print(self.pts_2d[idx])
        jac = np.zeros(cv_jac.shape)
        jac[:, 9:15] = cv_jac[:,:6]    #rotation translation vector
        jac[:, :4]   = cv_jac[:,6:10]  #intrinsic
        jac[:, 4:9]  = cv_jac[:,10:15] #distortion
        print(cv_jac)
        plt.imshow(cv_jac, interpolation='nearest', cmap=cm.Greys_r)
        plt.show()
        return frame_residual, jac

    def residual(self, x):
        frame_residuals = []
        self.last_jac = np.zeros((self.frame_num * self.point_num * 2, len(x.data)))
        for idx in range(self.frame_num):
            res, jac = self.frameResidual(x, idx)
            frame_residuals.append(res)
            offset = 9 # intrinsic = 4, distortion = 5, 4 + 5 = 9
            self.last_jac[idx * self.point_num * 2: (idx+1) * self.point_num * 2, : offset] = jac[:, :offset]
            self.last_jac[idx * self.point_num * 2: (idx+1) * self.point_num * 2, offset + idx * 6: offset + (idx + 1) * 6] = jac[:, offset:]

        self.last_x = x
        return np.hstack(frame_residuals)
    
    def jac(self, x):
        if x != self.last_x:
            self.residual(x)
        # print(self.last_jac[4:10,4:10])
        # plt.imshow(self.last_jac, interpolation='nearest', cmap=cm.Greys_r)
        # plt.show()
        return self.last_jac

    @staticmethod
    def update(x, update):
        dx = StateVector((len(update) - 9) / 6)
        dx.set_data(update)
        return StateVector.update(x, dx)


if __name__ == "__main__":
    np.set_printoptions(precision=5, linewidth=np.inf)
    sampler = CalibrationSampler(sample_num=2)
    sampler.camera.distortion = np.array([0.2, -0.1, 0, 0, 2])
    # sample_pts_2d = sampler.ProjectedPoints()
    # frame_num = 2
    
    problem = PinholeCalibrationProblem(sampler.BodyFramePoints(), sampler.ProjectedPoints())
    guess = StateVector(sampler.sample_num)
    guess.set_intrinsic(sampler.camera.intrinsic_array)
    # guess.distortion = np.array([0.] * 5)
    guess.distortion = sampler.camera.distortion
    # guess.frame_pose = sampler.tf_board_to_cam
    guess.frame_pose = sampler.tf_board_to_cam
    # print(guess.frame_pose)
    # quit()
    # print(guess.get_intrinsic())
    # quit()
    problem.solve(guess, verbose=2, step=1)