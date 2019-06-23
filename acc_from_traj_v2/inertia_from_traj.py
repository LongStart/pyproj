import numpy as np
from dsp import *
from lie_algebra import *

def AngleRateInBody(t, xyzw):
    angle_rate = AngleRate(t, xyzw)
    rot_global_to_body = R.from_quat(xyzw.transpose()).inv().as_dcm()
    return np.array([rot_global_to_body[i].dot(angle_rate.transpose()[i]) for i in range(len(t))]).transpose()

def AccelInIMU(t, xyz, xyzw, gravity, body_to_imu_xyz_xyzw):
    v_xyz = Derivative3d(t, xyz)
    a_xyz = (Derivative3d(t, v_xyz).transpose() + gravity).transpose()
    rot_global_to_body = R.from_quat(xyzw.transpose()).inv().as_dcm()
    rot_body_to_global = R.from_quat(xyzw.transpose()).as_dcm()
    angle_rate_xyz = AngleRateInBody(t, xyzw)
    angle_rate_hat = hat(angle_rate_xyz.transpose())
    angle_acc_xyz  = Derivative3d(t, angle_rate_xyz)
    angle_acc_hat = hat(angle_acc_xyz.transpose())

    euler_acc_from_global = np.array([rot_body_to_global[i].dot(angle_rate_hat[i]).dot(angle_rate_hat[i]).dot(body_to_imu_xyz_xyzw[:3]) for i in range(len(t))]).transpose()
    centripetal_acc_from_global = np.array([rot_body_to_global[i].dot(angle_acc_hat[i]).dot(body_to_imu_xyz_xyzw[:3]) for i in range(len(t))]).transpose()

    # all_acc_from_global = a_xyz + euler_acc_from_global + centripetal_acc_from_global
    # all_acc_from_global = euler_acc_from_global + centripetal_acc_from_global
    all_acc_from_global = a_xyz

    acc_in_body = np.array([rot_global_to_body[i].dot(all_acc_from_global.transpose()[i]) for i in range(len(t))]).transpose()
    acc_in_imu = StaticRotVec3d(body_to_imu_xyz_xyzw[3:], acc_in_body)
    acc_in_imu = np.vstack((t, acc_in_imu))
    return acc_in_imu

def InertiaFromTrajectory(t_xyz_xzyw, body_to_imu_xyz_xyzw, gravity):

    body_angle_rate_xyz = AngleRateInBody(t_xyz_xzyw[0], t_xyz_xzyw[4:])
    body_angle_rate_txyz = np.vstack((t_xyz_xzyw[0], body_angle_rate_xyz))
    imu_angle_rate_xyz = StaticTransformVec3d(body_to_imu_xyz_xyzw, body_angle_rate_xyz)
    imu_angle_rate_txyz = np.vstack((t_xyz_xzyw[0], imu_angle_rate_xyz))
    
    temporal_rot = np.array([[0,1,0],[0,0,1],[1,0,0]])
    imu_angle_rate_txyz[1:] = np.array([temporal_rot.dot(imu_angle_rate_txyz[1:].transpose()[i]) for i in range(len(t_xyz_xzyw[0]))]).transpose()
    imu_acceleration = AccelInIMU(t_xyz_xzyw[0], t_xyz_xzyw[1:4], t_xyz_xzyw[4:], gravity, body_to_imu_xyz_xyzw)
    return (imu_angle_rate_txyz, imu_acceleration)