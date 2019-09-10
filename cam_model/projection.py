import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

class CalibrationBoard(object):
    def __init__(self, position=[0,0,0.], orientation=[0,0,0,1.], size_w_h=[6,4], spacing=0.1):
        self.position = np.array(position)
        self.orientation = R.from_quat(orientation)
        self.size_w_h = size_w_h
        self.spacing = spacing

    def Points(self):
        point_mat = np.zeros((self.size_w_h[0]* self.size_w_h[1], 3))
        for i in range(self.size_w_h[0]):
            for j in range(self.size_w_h[1]):
                point_mat[self.size_w_h[1] * i + j] = self.orientation.apply(np.array([i * self.spacing, j * self.spacing, 0])) + self.position
        return point_mat



class PinholeCamera():
    def __init__(self, position=[0,0,0.], orientation=[0,0,0,1.], resolution=[640,480], intrinsic=np.array([500., 500, 320, 240]), distortion=np.zeros(5)):
        self.position = np.array(position)
        self.orientation = R.from_quat(orientation)
        self.intrinsic = np.identity(3)
        self.intrinsic[0,0] = intrinsic[0]
        self.intrinsic[1,1] = intrinsic[1]
        self.intrinsic[0,2] = intrinsic[2]
        self.intrinsic[1,2] = intrinsic[3]
        self.distortion = distortion
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
    board = CalibrationBoard(position=[-0.2, -0.2, 0])
    cam = PinholeCamera(position=[0, 0, -1], distortion=[0.2, -0.1, 0, 0, 2])
    board_image_corrected = cam.Project(board.Points(), distort=False)
    board_image = cam.Project(board.Points())

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect('equal')
    ax.scatter(*board.Points().T)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.quiver(*cam.Arrow(), length=0.5, normalize=False)

    fig_uv = plt.figure()
    plt.axis([0, cam.resolution[0], cam.resolution[1], 0])
    plt.plot(board_image.T[0], board_image.T[1], "*")
    plt.plot(board_image_corrected.T[0], board_image_corrected.T[1], "*")
    plt.grid(1)


    plt.show()

