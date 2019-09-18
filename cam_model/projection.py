import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import cv2 as cv
import matplotlib.cm as cm

class CalibrationBoard(object):
    def __init__(self, position=[0,0,0.], orientation=[0,0,0], size_w_h=[6,4], spacing=0.1):
        self.position = np.array(position)
        self.orientation = R.from_rotvec(orientation)
        self.size_w_h = size_w_h
        self.spacing = spacing
        self.center_local = np.array([(size_w_h[0] - 1) * spacing * 0.5, (size_w_h[1] - 1) * spacing * 0.5, 0])
        self.center_global = self.orientation.apply(self.center_local) + self.position

    def Points(self):
        pts = self.BodyFramePoints()
        pts_in_world = self.orientation.apply(pts) + self.position
        return pts_in_world

    def BodyFramePoints(self):
        point_mat = np.zeros((self.size_w_h[0]* self.size_w_h[1], 3))
        for i in range(self.size_w_h[0]):
            for j in range(self.size_w_h[1]):
                point_mat[self.size_w_h[1] * i + j] = np.array([i * self.spacing, j * self.spacing, 0])
        return point_mat

    def PoseInCameraFrame(self, cam_state):
        pose = Pose()
        pose.ang_p = (R.from_rotvec(-cam_state.ang_p) * self.orientation).as_rotvec()
        pose.lin_p = R.from_rotvec(-cam_state.ang_p).apply(self.position - cam_state.lin_p)
        return pose

class Pose():
    def __init__(self, lin_p=np.zeros(3), ang_p=np.zeros(3)):
        self.lin_p = lin_p
        self.ang_p = ang_p

    def vector6(self):
        return np.hstack([self.ang_p, self.lin_p])

def PinholeCameraDistortPoint(distortion, intrinsic,  point_in_uv):
    '''
    reference: https://www.mathworks.com/help/vision/ug/camera-calibration.html
    '''
    k1, k2, p1, p2, k3 = distortion
    f = np.array([intrinsic[0,0], intrinsic[1,1]])
    c = np.array([intrinsic[0,2], intrinsic[1,2]])
    x, y = (point_in_uv - c)/f
    r_2 = x**2 + y**2
    x_dist = x * (1 + k1 * r_2 + k2 * r_2 **2 + k3 * r_2 ** 3) + 2 * p1 * x*y + p2 * (r_2 + 2 * x**2)
    y_dist = y * (1 + k1 * r_2 + k2 * r_2 **2 + k3 * r_2 ** 3) + 2 * p2 * x*y + p1 * (r_2 + 2 * y**2)
    return np.array([x_dist, y_dist]) * f + c

def PinholeCameraProjectPoint(points, cam_rotation, cam_position, intrinsic_mat):
    points_in_cam = R.from_rotvec(cam_rotation).inv().apply(points - cam_position)
    points_in_uv = np.array([intrinsic_mat.dot(p)[:2]/p[2] for p in points_in_cam])
    return points_in_uv

def GetPointInROI(points, width, height, pt_size=0.4):
    result = {}
    for i in range(len(points)):
        if width[0] - pt_size< points[i][0] <= width[1] + pt_size and height[0] - pt_size < points[i][1] <= height[1] + pt_size:
            result[i] = points[i]
    return result

class RadTanPinhole():
    def __init__(self, intrinsic=np.array([500., 500, 320, 240]), distortion=[0.]*5):
        self.intrinsic_array = intrinsic
        self.distortion = np.array(distortion)

    @property
    def intrinsic_mat(self):
        mat = np.identity(3)
        mat[0,0] = self.intrinsic_array[0]
        mat[1,1] = self.intrinsic_array[1]
        mat[0,2] = self.intrinsic_array[2]
        mat[1,2] = self.intrinsic_array[3]
        return mat.astype('float32')

class PinholeCamera():
    def __init__(self, resolution=[640,480], model=RadTanPinhole()):
        self.model = model
        self.resolution = resolution

    def Distort(self, points_in_uv):
        return np.vstack([PinholeCameraDistortPoint(self.model.distortion, self.model.intrinsic_mat, p) for p in points_in_uv])

    def Project(self, points, cam_pose, distort=True):
        # assert(cam_pose_rt.shape == (2,3))
        # cam_pose_rt = np.zeros((2,3))
        # if cam_pose_rt_in.shape == (2,3):
        #     cam_pose_rt = cam_pose_rt_in
        # elif cam_pose_rt_in.shape == (2,3,3):
        #     cam_pose_rt = cam_pose_rt_in[:, 0, :]
        # else:
        #     raise TypeError
        points_in_uv = PinholeCameraProjectPoint(points, cam_pose.ang_p, cam_pose.lin_p, self.model.intrinsic_mat)
        if distort:
            points_in_uv = self.Distort(points_in_uv)
        return points_in_uv

    @staticmethod
    def Arrow(cam_pose):
        rot = R.from_rotvec(cam_pose.ang_p)
        return np.hstack((cam_pose.lin_p, rot.apply([0,0,1])))

if __name__ == "__main__":
    np.set_printoptions(precision=10, linewidth=np.inf)
    board = CalibrationBoard(position=[0., 0., 0], orientation=[math.pi/2,0.,0.], size_w_h=[2,2])
    # cam = PinholeCamera(position=[0, 0.2, -1], orientation=[0.,0.,0.], distortion=[0.2, -0.1, 0, 0, 2])

    cam = PinholeCamera(model=RadTanPinhole(distortion=[0.2, -0.1, 0, 0, 2]))
    # cam_pose = np.array([[-math.pi/2,0.,0.],[0.05, -0.8, 0.05]])
    cam_pose = Pose(lin_p=np.array([0.05, -0.8, 0.05]), ang_p=np.array([-math.pi/2,0.,0.]))
    board_image_corrected = cam.Project(board.Points(), cam_pose, distort=False)
    board_image = cam.Project(board.Points(), cam_pose)

    cv_board_img = np.array([board_image], dtype=np.float32)
    cv_board_pts = np.array([board.BodyFramePoints()], dtype=np.float32)
    ptsOut = cv.undistortPoints(cv_board_img, cam.model.intrinsic_mat, cam.model.distortion)
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(cv_board_pts, cv_board_img, tuple(cam.resolution[::-1]), None, None)
    # rotvec_board_to_cam = (cam.orientation.inv() * board.orientation).as_rotvec()
    # tran_board_to_cam = cam.orientation.inv().apply(board.position - cam.position)

    if 0:
        print("project points input: ")
        print(board.BodyFramePoints())
        print(board.PoseInCameraFrame(cam_pose).ang_p)
        print(board.PoseInCameraFrame(cam_pose).lin_p)
        print(cam.model.intrinsic_mat)
        print(cam.model.distortion)
    imgpts0, jac = cv.projectPoints(board.BodyFramePoints(), board.PoseInCameraFrame(cam_pose).ang_p, board.PoseInCameraFrame(cam_pose).lin_p, cam.model.intrinsic_mat, cam.model.distortion)
    # print(imgpts0.ravel())
    eps = 1e-8
    tran_inc = board.PoseInCameraFrame(cam_pose).lin_p + np.array([0,  0, eps])
    # left_rot_inc = (R.from_rotvec([ 0, 0,eps]) * R.from_rotvec(rotvec_board_to_cam)).as_rotvec()
    left_rot_inc = (R.from_rotvec([ 0,eps, 0]) * R.from_rotvec(board.PoseInCameraFrame(cam_pose).ang_p)).as_rotvec()
    # left_rot_inc = (R.from_rotvec([ eps, 0, 0]) * R.from_rotvec(rotvec_board_to_cam)).as_rotvec()
    right_rot_inc = (R.from_rotvec(board.PoseInCameraFrame(cam_pose).ang_p) * R.from_rotvec([ 0,eps, 0])).as_rotvec()
    imgpts1, jac_temp = cv.projectPoints(board.BodyFramePoints(), left_rot_inc, board.PoseInCameraFrame(cam_pose).lin_p, cam.model.intrinsic_mat, cam.model.distortion)
    # imgpts = imgpts.reshape((24,2))
    dp_dx = (imgpts1 - imgpts0) / eps
    # print("num: {},\nana: {}".format(dp_dx.ravel(), jac[:, 0:2]))

    imgpts = imgpts0
    # print(imgpts.shape)
    residual = board_image.ravel() - imgpts.ravel()
    print("residual: {}".format(residual))


    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect('equal')
    ax.scatter(*board.Points().T)
    # ax.scatter([0],[0],[0], marker='+', color='r')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.quiver(*PinholeCamera.Arrow(cam_pose), length=0.5, normalize=False)

    fig_uv = plt.figure()
    plt.axis([0, cam.resolution[0], cam.resolution[1], 0])
    # plt.plot(board_image.T[0], board_image.T[1], "*")
    plt.plot(board_image_corrected.T[0], board_image_corrected.T[1], "*")
    plt.plot(imgpts.T[0], imgpts.T[1], ".")
    plt.plot([320], [240], "+", markersize=15)
    plt.xlabel('U')
    plt.ylabel('V')
    plt.grid(1)


    plt.show()

