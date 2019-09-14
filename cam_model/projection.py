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
        point_mat = np.zeros((self.size_w_h[0]* self.size_w_h[1], 3))
        for i in range(self.size_w_h[0]):
            for j in range(self.size_w_h[1]):
                point_mat[self.size_w_h[1] * i + j] = self.orientation.apply(np.array([i * self.spacing, j * self.spacing, 0])) + self.position
        return point_mat

    def BodyFramePoints(self):
        point_mat = np.zeros((self.size_w_h[0]* self.size_w_h[1], 3))
        for i in range(self.size_w_h[0]):
            for j in range(self.size_w_h[1]):
                point_mat[self.size_w_h[1] * i + j] = np.array([i * self.spacing, j * self.spacing, 0])
        return point_mat

class PinholeCamera():
    def __init__(self, position=[0,0,0.], orientation=[0,0,0], resolution=[640,480], intrinsic=np.array([500., 500, 320, 240]), distortion=[0.]*5):
        self.position = np.array(position)
        self.orientation = R.from_rotvec(orientation)
        self.intrinsic_array = intrinsic
        self.intrinsic = np.identity(3)
        self.intrinsic[0,0] = intrinsic[0]
        self.intrinsic[1,1] = intrinsic[1]
        self.intrinsic[0,2] = intrinsic[2]
        self.intrinsic[1,2] = intrinsic[3]
        self.distortion = np.array(distortion)
        self.resolution = resolution

    @staticmethod
    def DistortPoint(distortion, intrinsic,  point_in_uv):
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

    def Distort(self, points_in_uv):
        return np.vstack([PinholeCamera.DistortPoint(self.distortion, self.intrinsic, p) for p in points_in_uv])

    def Project(self, points, distort=True):
        points_in_cam = self.orientation.inv().apply(points - self.position)
        points_in_uv = np.array([self.intrinsic.dot(p)[:2]/p[2] for p in points_in_cam])
        if distort:
            points_in_uv = self.Distort(points_in_uv)
        return points_in_uv

    def Arrow(self):
        return np.hstack((self.position, self.orientation.apply([0,0,1])))

if __name__ == "__main__":
    np.set_printoptions(precision=10, linewidth=np.inf)
    board = CalibrationBoard(position=[0., 0., 0], orientation=[math.pi/2,0.,0.], size_w_h=[2,2])
    # cam = PinholeCamera(position=[0, 0.2, -1], orientation=[0.,0.,0.], distortion=[0.2, -0.1, 0, 0, 2])
    cam = PinholeCamera(position=[0.05, -0.8, 0.05], orientation=[-math.pi/2,0.,0.], distortion=[0.2, -0.1, 0, 0, 2])
    board_image_corrected = cam.Project(board.Points(), distort=False)
    board_image = cam.Project(board.Points())

    cv_board_img = np.array([board_image], dtype=np.float32)
    cv_board_pts = np.array([board.BodyFramePoints()], dtype=np.float32)
    ptsOut = cv.undistortPoints(cv_board_img, cam.intrinsic, cam.distortion)
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(cv_board_pts, cv_board_img, tuple(cam.resolution[::-1]), None, None)
    rotvec_board_to_cam = (cam.orientation.inv() * board.orientation).as_rotvec()
    tran_board_to_cam = cam.orientation.inv().apply(board.position - cam.position)

    if 0:
        print("project points input: ")
        print(board.BodyFramePoints())
        print(rotvec_board_to_cam)
        print(tran_board_to_cam)
        print(cam.intrinsic)
        print(cam.distortion)
    imgpts0, jac = cv.projectPoints(board.BodyFramePoints(), rotvec_board_to_cam, tran_board_to_cam, cam.intrinsic, cam.distortion)
    # print(imgpts0.ravel())
    eps = 1e-8
    tran_inc = tran_board_to_cam + np.array([0,  0, eps])
    # left_rot_inc = (R.from_rotvec([ 0, 0,eps]) * R.from_rotvec(rotvec_board_to_cam)).as_rotvec()
    left_rot_inc = (R.from_rotvec([ 0,eps, 0]) * R.from_rotvec(rotvec_board_to_cam)).as_rotvec()
    # left_rot_inc = (R.from_rotvec([ eps, 0, 0]) * R.from_rotvec(rotvec_board_to_cam)).as_rotvec()
    right_rot_inc = (R.from_rotvec(rotvec_board_to_cam) * R.from_rotvec([ 0,eps, 0])).as_rotvec()
    imgpts1, jac_temp = cv.projectPoints(board.BodyFramePoints(), left_rot_inc, tran_board_to_cam, cam.intrinsic, cam.distortion)
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

    ax.quiver(*cam.Arrow(), length=0.5, normalize=False)

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

