import bsplines
import numpy as np
import sm
import numpy
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

def SightOrientation(from_point, to_point, keep_horizon=True):
    z = (to_point - from_point)
    z /= np.linalg.norm(z)
    x = np.zeros(3)
    x[0] = z[1]
    x[1] = -z[0]
    x /= np.linalg.norm(x)
    if np.cross(z, x)[2] > 0:
        x *= -1.
    y = np.cross(z, x)
    return R.from_dcm(np.vstack([x,y,z]).T)

class TargetOrientationPoseSpline(object):
    def __init__(self, order=4, target_point=[0.,0,0], ctrl_point_num=10, time=10., random_range=[[0,0,0.],[1.,1.,1.]]):
        self.target_point = np.array(target_point)
        self.bsp = bsplines.BSplinePose(4, sm.RotationVector())
        self.ctrl_point_num = ctrl_point_num
        random_range = np.array(random_range) + self.target_point
        print(random_range)
        
        self.curve = self.TargetOrientedPose(ctrl_point_num, random_range)
        self.time = time

        times = np.linspace(0, time, ctrl_point_num)
        curve_in = np.array(self.curve)
        curve_in[3:] *= -1
        self.bsp.initPoseSplineSparse(times, curve_in, int(ctrl_point_num * 3.), 1e-5)
        # self.bsp.initPoseSpline3(times, self.curve, int(ctrl_point_num * 3.), 1e-5)

    def TargetOrientedPose(self, point_num, random_range):
        curve = []
        random_range = np.array(random_range)
        # print(self.ctrl_point_num)
        for i in range(self.ctrl_point_num):
            pose = np.zeros(6)
            pose[:3] = (random_range[1] - random_range[0]) * numpy.random.random(3) + random_range[0]
            pose[3:] = SightOrientation(pose[:3], self.target_point).as_rotvec()
            curve.append(pose)
        curve = np.vstack(curve).T
        return curve

    def ControlPoses(self):
        return self.curve.T

    @staticmethod
    def PoseToArrow(xyz_rot_xyz):
        from copy import deepcopy
        plt_arrows = []
        for pose in xyz_rot_xyz:
            plt_arrow = deepcopy(pose)
            plt_arrow[3:] = R.from_rotvec(pose[3:]).apply(np.array([0,0,1.]))
            plt_arrows.append(plt_arrow)
        return np.vstack(plt_arrows).T

    def VizControlPoses(self):
        return TargetOrientationPoseSpline.PoseToArrow(self.ControlPoses())

    def VizPositionCurve(self, sample_number=500):
        return np.vstack([self.bsp.position(p) for p in np.linspace(0, self.time, 500)]).T

    def VizPoseCurve(self, sample_number=500):
        # positions = np.vstack([self.bsp.position(p) for p in np.linspace(0, self.time, 500)]).T
        # orientations = np.vstack([self.bsp.orientation(p) for p in np.linspace(0, self.time, 500)]).T
        xyz_rot_xyz = np.vstack([ \
            np.hstack([self.bsp.position(t), R.from_dcm(self.bsp.orientation(t)).as_rotvec()]) \
                for t in np.linspace(0, self.time, sample_number)])
        return TargetOrientationPoseSpline.PoseToArrow(xyz_rot_xyz)

if __name__ == "__main__":
    target_traj = TargetOrientationPoseSpline(ctrl_point_num=5, time=30, random_range=[[0.5,0,-1],[1,.5,1]])
    positions = target_traj.VizPositionCurve()
    poses = target_traj.VizPoseCurve(100)
    control_points = target_traj.VizControlPoses()
    # print(control_points[3:,0])
    # r0 = R.from_dcm(target_traj.bsp.orientation(0.0))
    # print(r0.as_rotvec())
    # print(np.linalg.norm(r0.as_rotvec()))
    # quit()

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect('equal')
    ax.plot(*positions)
    ax.scatter(0,0,0, color='g')
    ax.quiver(*control_points, length=0.2, normalize=False, color='r')
    ax.quiver(*poses, length=.2, normalize=False, color='y')
    ax.set_xlim(-3, 3)
    ax.set_ylim(-3, 3)
    ax.set_zlim(-3, 3)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()