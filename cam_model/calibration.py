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

class OptimizationMask(object):
    def __init__(self, state_space_length, residual_length, frame_num):
        self.len_state = state_space_length
        self.len_residual = residual_length
        self.frame_num = frame_num
        self.shape_state = (self.len_state,)
        self.shape_jac = (self.len_residual, self.len_state)
        self.mode = 'pose'
        # self.mode = 'full'
        self.callback_dict = {
            'pose': (self.extractPose, self.composeByPose),
            'translation': (self.extractTranslation, self.composeByTranslation),
            'rotation': (self.extractRotation, self.composeByRotation),
            'intrinsic': (self.extractIntrisic, self.composeByIntrisic),
            'distortion': (self.extractDistortion, self.composeByDistortion),
            'full': (self.extractFull, self.composeByFull)}

    #extract jacobian
    def extractDistortion(self, val):
        assert(val.shape == self.shape_jac)
        return val[:,4:9]

    #compose state vector
    def composeByDistortion(self, val):
        assert(val.shape == (5,))
        return np.hstack([np.zeros(4), val, np.zeros(self.len_state - 9)])

    #extract jacobian
    def extractIntrisic(self, val):
        assert(val.shape == self.shape_jac)
        return val[:,:4]

    #compose state vector
    def composeByIntrisic(self, val):
        assert(val.shape == (4,))
        return np.hstack([val, np.zeros(self.len_state - len(val))])

    #extract jacobian
    def extractPose(self, val):
        assert(val.shape == self.shape_jac)
        return val[:,9:]

    #compose state vector
    def composeByPose(self, val):
        assert(val.shape == (self.len_state - 9,))
        return np.hstack([np.zeros(9), val])

    #extract jacobian
    def extractTranslation(self, val):
        assert(val.shape == self.shape_jac)
        return np.hstack([val[:, 9 + 6 * i + 3: 9 + 6 * i + 6] for i in range(self.frame_num)])

    #compose state vector
    def composeByTranslation(self, val):
        assert(val.shape == (self.frame_num * 3,))
        poses = np.hstack([np.zeros((self.frame_num, 3)), val.reshape((self.frame_num, 3))]).ravel()
        return np.hstack([np.zeros(9), poses])

    #extract jacobian
    def extractRotation(self, val):
        assert(val.shape == self.shape_jac)
        return np.hstack([val[:, 9 + 6 * i: 9 + 6 * i + 3] for i in range(self.frame_num)])

    #compose state vector
    def composeByRotation(self, val):
        assert(val.shape == (self.frame_num * 3,))
        poses = np.hstack([val.reshape((self.frame_num, 3)), np.zeros((self.frame_num, 3))]).ravel()
        return np.hstack([np.zeros(9), poses])

    #extract jacobian
    def extractFull(self, val):
        # print(val.shape)
        # print(self.shape_jac)
        assert(val.shape == self.shape_jac)
        return val

    #compose state vector
    def composeByFull(self, val):
        assert(val.shape == (self.len_state, ))
        return val

    def Extract(self, val):
        return self.callback_dict[self.mode][0](val)

    def Compose(self, val):
        return self.callback_dict[self.mode][1](val)



class PinholeCalibrationProblem(Problem):
    def __init__(self, pts_3d, pts_2d):
        assert(pts_3d.shape[:2] == pts_2d.shape[:2])
        self.pts_3d = pts_3d
        self.pts_2d = pts_2d
        self.frame_num, self.point_num = self.pts_2d.shape[:2]
        self.last_x = StateVector(self.frame_num)
        self.last_jac = np.array([0.] * self.point_num * 2)
        self.last_res = np.zeros(self.point_num * 2)
        self.mask = OptimizationMask(4 + 5 + 6 * self.frame_num, self.frame_num * self.point_num * 2, self.frame_num)

    def frameRotationJac(self, x, idx):
        eps = 1e-5
        eps_mat = np.identity(3) * eps
        project_incs = np.array([np.zeros(self.point_num * 2)] * 3)
        project_base = np.zeros(self.point_num * 2)
        project_base, jac = cv.projectPoints(self.pts_3d[idx], x.rotation(idx), x.translation(idx), x.get_intrinsic(), x.distortion)
        for i in range(3):
            rot_inc = (R.from_rotvec(eps_mat[i]) * R.from_rotvec(x.rotation(idx))).as_rotvec()
            proj, jac = cv.projectPoints(self.pts_3d[idx], rot_inc, x.translation(idx), x.get_intrinsic(), x.distortion)
            project_incs[i] = proj.ravel()
        jac = (project_incs - project_base.ravel()).T / eps
        # print(jac)
        return jac



    def frameResidual(self, x, idx):
        if 0:
            print("project points input: ")
            print(self.pts_3d[idx])
            print(x.rotation(idx))
            print(x.translation(idx))
            print(x.get_intrinsic())
            print(x.distortion)
        project_2d, cv_jac = cv.projectPoints(self.pts_3d[idx], x.rotation(idx), x.translation(idx), x.get_intrinsic(), x.distortion.astype("float32"))
        # print(project_2d.ravel())
        frame_residual = project_2d.ravel() - self.pts_2d[idx].ravel()
        # print("cv_jac: ")
        # print(cv_jac[:, 3:6])

        jac = np.zeros(cv_jac.shape)
        # jac[:, 9:12] = cv_jac[:,:3]    #rotation vector
        jac[:, 9:12] = self.frameRotationJac(x, idx)    #rotation vector
        jac[:, 12:15] = cv_jac[:,3:6]    #translation vector
        jac[:, :4]   = cv_jac[:,6:10]  #intrinsic
        jac[:, 4:9]  = cv_jac[:,10:15] #distortion

        return frame_residual, jac

    def residual(self, x):
        if x == self.last_x:
            return self.last_res

        frame_residuals = []
        self.last_jac = np.zeros((self.frame_num * self.point_num * 2, len(x.data)))
        for idx in range(self.frame_num):
            res, jac = self.frameResidual(x, idx)
            frame_residuals.append(res)
            offset = 9 # intrinsic = 4, distortion = 5, 4 + 5 = 9
            self.last_jac[idx * self.point_num * 2: (idx+1) * self.point_num * 2, : offset] = jac[:, :offset]
            self.last_jac[idx * self.point_num * 2: (idx+1) * self.point_num * 2, offset + idx * 6: offset + (idx + 1) * 6] = jac[:, offset:]

        self.last_x = x
        self.last_res = np.hstack(frame_residuals)
        # plt.imshow(self.last_jac, interpolation='nearest', cmap=cm.Greys_r)
        # plt.show()
        # print("self.last_jac: ")
        # print(self.last_jac[:, 9:12])
        # j = self.last_jac[:, -3:]
        # print(j.T.dot(self.last_res) + self.last_res.dot(j))
        return self.last_res

    def jac(self, x):
        while x != self.last_x:
            self.residual(x)
        # print(self.last_jac[4:10,4:10])
        # plt.imshow(self.last_jac, interpolation='nearest', cmap=cm.Greys_r)
        # plt.show()
        return self.mask.Extract(self.last_jac)

    def update(self, x, update_in):
        update = self.mask.Compose(update_in)
        dx = StateVector((len(update) - 9) / 6)
        dx.set_data(update)
        return StateVector.update(x, dx)

if __name__ == "__main-__":
    mask = OptimizationMask(11,1)
    a = mask.Extract(np.array([[2.]* 11]))
    b = mask.Compose(np.array([1,2]))
    print(a)
    print(b)

if __name__ == "__main__":
    np.set_printoptions(precision=5, linewidth=np.inf)

    sampler = CalibrationSampler(sample_num=20, cam_distortion=[0.2, -0.3, 0., 0., 2.])
    # sampler.camera.distortion = np.array([0., 5., 0., 0.2, 2.])
    # sample_pts_2d = sampler.ProjectedPoints()
    # frame_num = 2

    problem = PinholeCalibrationProblem(sampler.BodyFramePoints(), sampler.ProjectedPoints())
    guess = StateVector(sampler.sample_num)
    guess.set_intrinsic(sampler.camera.intrinsic_array + 5)
    # guess.distortion = sampler.camera.distortion
    guess.distortion = np.array([0.] * 5)
    guess.frame_pose = sampler.tf_board_to_cam + 0.5
    problem.solve(guess, verbose=2, step=25)

    if 0:
        ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
                sampler.BodyFramePoints().astype('float32'),
                sampler.ProjectedPoints().astype('float32'),
                tuple(sampler.camera.resolution[::-1]), None, None)
        print(dist)