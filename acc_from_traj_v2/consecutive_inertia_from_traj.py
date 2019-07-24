import numpy as np
from core.dsp import *
from core.lie_algebra import *
import bsplines
from scipy.spatial.transform import Rotation as R

class BTrajectory():
    def __init__(self, translation_spl, rotation_spl):
        self.pos_spl = translation_spl
        self.quat_spl = rotation_spl

# def CreateAngleRateFunc(quat_spl):
#     def func(t):


def Continualization(t_xyz_xyzw, lamb=1e-5, max_time=50000):
    order = 4
    pos_spl = bsplines.EuclideanBSpline(order, 3)
    pos_spl.initUniformSpline(t_xyz_xyzw[0], t_xyz_xyzw[1:4], max_time, lamb)
    quat_spl = bsplines.UnitQuaternionBSpline(order)
    quat_spl.initUniformSpline(t_xyz_xyzw[0], t_xyz_xyzw[4:], max_time, lamb)
    return BTrajectory(pos_spl, quat_spl)


def AngleRateInIMUFunction(quat_spl, body_to_imu_xyz_xyzw):
    def func(t):
        q_body_to_imu = R.from_quat(body_to_imu_xyz_xyzw[3:7])
        temporal_rot = R.from_dcm([[0,1,0],[0,0,1],[1,0,0]])
        q_body_to_imu = temporal_rot * q_body_to_imu
        return np.array([q_body_to_imu.apply(quat_spl.getEvaluatorAt(tt).evalAngularVelocity()) for tt in t]).transpose()
    return func

# def AngleRateInBodyFunc(quat_spl):
#     def f(t):
#         return np.array([quat_spl.getEvaluatorAt(tt).evalAngularVelocity() for tt in t]).transpose()
#     return f

# def EulerAccelInGlobalFunc(quat_body_to_global_func, angle_rate_in_body_func, body_to_imu_xyz_xyzw):
#     body_to_imu_translation = body_to_imu_xyz_xyzw[:3]
    
#     def f(t):
#         angle_rate_hat = hat(angle_rate_in_body_func(t).transpose())
#         rot_body_to_global = R.from_quat(quat_body_to_global_func(t))
#         euler_acc_in_global = np.array([rot_body_to_global[i].dot(angle_rate_hat[i]).dot(angle_rate_hat[i]).dot(body_to_imu_translation) for i in range(len(t))]).transpose()
#         return euler_acc_in_global
#     return f

# def MotionAccelInGlobalFunc(pos_spl, gravity):
#     def f(t):
#         return (pos_spl.evalD(t, 2) + gravity).transpose()
#     return f

# def EmptyAccelFunc(t):
#         return np.zeros(len(t), 3)

# def AllAccelInGlobal(motion_f, euler_f=EmptyAccelFunc, centripetal_f=EmptyAccelFunc):
#     def f(t):
#         return euler_f(t) + motion_f(t) + centripetal_f(t)
#     return f 

def AngleRateInBody(t, xyzw):
    angle_rate = AngleRate(t, xyzw)
    rot_global_to_body = R.from_quat(xyzw.transpose()).inv().as_dcm()
    return np.array([rot_global_to_body[i].dot(angle_rate.transpose()[i]) for i in range(len(t))]).transpose()

def EulerAccel(t, rot_body_to_global, angle_rate_in_body, trans_body_to_imu):
    angle_rate_hat = hat(angle_rate_in_body)
    return np.array([rot_body_to_global[i].dot(angle_rate_hat[i]).dot(angle_rate_hat[i]).dot(trans_body_to_imu) for i in range(len(t))])

def CentripetalAccel(t, rot_body_to_global, angle_acc_in_body, trans_body_to_imu):
    angle_acc_hat = hat(angle_acc_in_body)
    return np.array([rot_body_to_global[i].dot(angle_acc_hat[i]).dot(trans_body_to_imu) for i in range(len(t))])


def AccelInIMUFunc(btraj, body_to_imu_xyz_xyzw, gravity):
    rot_body_to_imu = R.from_quat(body_to_imu_xyz_xyzw[3:])
    trans_body_to_imu = body_to_imu_xyz_xyzw[:3]
    
    def f(t):
        motion_acc_in_global = np.array([btraj.pos_spl.evalD(tt, 2) + gravity for tt in t])
        rot_global_to_body = R.from_quat([btraj.quat_spl.eval(tt) for tt in t]).inv()
        all_acc_in_global = motion_acc_in_global
        # all_acc_in_global = EulerAccel(t, rot_global_to_body.inv(), )
        all_acc_in_imu = rot_body_to_imu.apply(rot_global_to_body.apply(all_acc_in_global))
        return all_acc_in_imu.transpose()

    return f

def InertiaFuncFromTrajectory(t_xyz_xyzw, body_to_imu_xyz_xyzw, gravity):
    btraj = Continualization(t_xyz_xyzw)
    angle_rate_f = AngleRateInIMUFunction(btraj.quat_spl, body_to_imu_xyz_xyzw)
    acc_f = AccelInIMUFunc(btraj, body_to_imu_xyz_xyzw, gravity)
    return angle_rate_f, acc_f
        
    
