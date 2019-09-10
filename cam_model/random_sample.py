import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

from projection import *
from random_pose_spline import *

if __name__ == "__main__":
    board = CalibrationBoard(position=[-0.25, 0, -0.15], orientation=R.from_rotvec([math.pi / 2, 0, 0]).as_quat())
    # cam = PinholeCamera(position=[0, 0, -0.5], distortion=[0.2, -0.1, 0, 0, 2])
    # board_image_corrected = cam.Project(board.Points(), distort=False)
    # board_image = cam.Project(board.Points())

    target_traj = TargetOrientationPoseSpline(ctrl_point_num=20, time=50, random_range=[[-1,-.7, -1],[1,-2, 1]])
    positions = target_traj.VizPositionCurve()
    control_points = target_traj.VizControlPoses()

    animation = True

    if not animation:

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_aspect('equal')
        ax.scatter(*board.Points().T)
        ax.scatter([0], [0], color='g')
        ax.plot(*positions, color='r')
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.set_zlim(-3, 3)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        plt.show()

    else:
        sample_t = np.linspace(0, target_traj.time, 200)
        # board_images_undist = []
        board_images = []
        for t in sample_t:
            cam_position = target_traj.bsp.position(t)
            cam_oriantation_q = R.from_dcm(target_traj.bsp.orientation(t)).as_quat()
            cam = PinholeCamera(position=cam_position, orientation=cam_oriantation_q, distortion=[0.2, -0.1, 0, 0, 2])
            # board_images_undist.append(cam.Project(board.Points(), distort=False))
            board_images.append((cam.Project(board.Points()), cam.Project(board.Points(), distort=False)))

        for image_undist, image in board_images:
            fig_uv = plt.clf()
            plt.axis([0, cam.resolution[0], cam.resolution[1], 0])
            plt.plot(image_undist.T[0], image_undist.T[1], "*")
            plt.plot(image.T[0], image.T[1], "*")
            plt.grid(1)
            plt.pause(1e-8)


