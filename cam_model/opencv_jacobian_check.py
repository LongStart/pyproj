import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import cv2 as cv
import matplotlib.cm as cm
from projection import *

if __name__ == "__main__":
    np.set_printoptions(precision=10, linewidth=np.inf)
    board = CalibrationBoard(position=[0., 0., 0], orientation=[math.pi/2,0.,0.], size_w_h=[3,2])
    cam = PinholeCamera(position=[0.1, -0.8, 0.05], orientation=[-math.pi/2,0.,0.], distortion=[0.2, -0.1, 0, 0, 2])
    board_image = cam.Project(board.Points())

    rotvec_board_to_cam = (cam.orientation.inv() * board.orientation).as_rotvec()
    tran_board_to_cam = cam.orientation.inv().apply(board.position - cam.position)
    eps = 1e-5

    if 0:
        print("project points input: ")
        print(board.BodyFramePoints())
        print(rotvec_board_to_cam)
        print(tran_board_to_cam)
        print(cam.intrinsic)
        print(cam.distortion)
    imgpts0, jac = cv.projectPoints(board.BodyFramePoints(), rotvec_board_to_cam, tran_board_to_cam, cam.intrinsic, cam.distortion)
    # print(imgpts0.ravel())

    dist_inc_mat = np.identity(5) * eps
    jac_num = np.ndarray((board.size_w_h[0] * board.size_w_h[1] * 2, 5))
    for i in range(5):
        imgpts_inc, jac_tmp = cv.projectPoints(board.BodyFramePoints(), rotvec_board_to_cam, tran_board_to_cam, cam.intrinsic, cam.distortion + dist_inc_mat[i])
        jac_num.T[i] = (imgpts_inc.ravel() - imgpts0.ravel()) / eps

    imgpts1, jac_tmp = cv.projectPoints(board.BodyFramePoints(), rotvec_board_to_cam, tran_board_to_cam, cam.intrinsic, cam.distortion + np.array([0., 1e-2, 0, 0, 0]))

    # redisual = board_image.ravel() - imgpts0.ravel()
    redisual = imgpts0.ravel() - imgpts1.ravel()

    print('jac num: ')
    print(jac_num)
    print('jac ana: ')
    print(jac[:, 10:])

    print('residual: ')
    print(redisual)

