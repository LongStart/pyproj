import rospy
import rosbag
import numpy as np
from sys import argv
# from offline_tf import OfflineTfBuffer
from tf.transformations import rotation_from_matrix
from tf.transformations import quaternion_matrix
import PlotCollection
from add_3axis_figure import *
from ros_io import *
from signal3d import Signal3d
from scipy.spatial.transform import Rotation as R

def GenerateGlobalGravity(t, xyz):
    # g = np.array([value] * len(t))
    x = np.array([xyz[0]] * len(t))
    y = np.array([xyz[1]] * len(t))
    z = np.array([xyz[2]] * len(t))
    return np.array([t, x, y, z])

def CorrectBiasedStamp(ts, threashold=0.7):
    dts = ts[1:] - ts[0:-1]
    dt_mean = dts.mean()
    dts_err = dts - dt_mean
    err_forward   = dts_err[:-1]
    err_backback  = dts_err[1:]
    comm = np.abs(err_forward + err_backback) / 2
    diff = np.abs(err_forward - err_backback) / 2
    cnt = 0
    for i in range(len(err_forward)):
        if diff[i] > 0.3 * dt_mean and comm[i] < 0.1 * dt_mean :
            idx_ts = i+1
            # print('biased stamp: {0}, cnt: {1}'.format(ts[idx_ts], cnt))
            # cnt += 1
            ts[idx_ts] = ts[idx_ts - 1] + dt_mean

if __name__ == '__main__':
    if(len(argv) < 3):
        print("Example: python acc_from_traj.py bag_path dataset_name")
        quit()
    bag_filename = argv[1]
    dataset_name = argv[2]
    groundtruth_topic_name = ''
    imu_topic_name = ''
    if dataset_name == 'euroc':
        groundtruth_topic_name = '/vicon/firefly_sbx/firefly_sbx'
        imu_topic_name = '/imu0'
        groundtruth_frame_name = "world"
        body_frame_name = "vicon/firefly_sbx/firefly_sbx"
    else:
        print('unknown dataset')
        quit()

    transform_msgs = ReadTopicMsg(bag_filename, groundtruth_topic_name)

    raw_position = PositionFromTransformStamped(transform_msgs)
    CorrectBiasedStamp(raw_position[0])

    pos_world = Signal3d(raw_position)
    vel_world = pos_world.Derivative()
    acc_world = vel_world.Derivative()

    
    # quit()

    body_to_world = Signal3d(RotationFromTransformStamped(transform_msgs))
    world_to_body = body_to_world*(-1)

    acc_body = acc_world.Rotate(world_to_body)
    # ave_acc_body = acc_body.Midfilter(kernel_size=19)
    # ave_acc_body = acc_body.MovingAverage(kernel_size=19)
    ave_acc_body = acc_body.LFilter(0.06)
    
    imu_msgs = ReadTopicMsg(bag_filename, imu_topic_name)
    acc_imu = Signal3d(AccelerationFromIMU(imu_msgs))
    rotate_imu_to_camera = R.from_dcm([[0,1,0],[0,0,1],[1,0,0]])
    acc_imu = acc_imu.Rotate(rotate_imu_to_camera.as_rotvec())
    # angle_rate_imu = Signal3d(AngleRateFromIMU(imu_msgs))
    # ave_acc_imu = acc_imu.Midfilter(kernel_size=19)
    # ave_acc_imu = acc_imu.MovingAverage(kernel_size=19)

    ave_num = 100
    gravity_est_raw = sum(acc_imu.xyz().transpose()[0:ave_num+1])/ave_num
    print(gravity_est_raw)
    gravity_est = rotate_imu_to_camera.as_dcm().dot(gravity_est_raw)
    

    #gravity
    # g_world = Signal3d(GenerateGlobalGravity(acc_imu.t()))
    # g_body = g_world.Rotate(world_to_body)
    # g_world = Signal3d(GenerateGlobalGravity(acc_imu.t(), gravity_est))
    g_world = Signal3d(GenerateGlobalGravity(acc_imu.t(), np.array([9.8, 0, 0])))
    g_body = g_world.Rotate(world_to_body)
    # acc_imu = acc_imu - g_body
    ave_acc_imu = acc_imu.LFilter(0.02)
    acc_bias = ave_acc_body - ave_acc_imu 

    plotter = PlotCollection.PlotCollection("My window name")
    pos = {'pos_w': pos_world.data}
    vel = {'vel': vel_world.data}
    acc = {
        # 'acc_b': acc_body.data, 
        'ave_acc_b': ave_acc_body.data,
        'ave_acc_imu': ave_acc_imu.data, 
        # 'acc_imu': acc_imu.data
        }
    acc_bias = {'acc_bias': acc_bias.data}
    # angle_rate = {'angle_rate_imu': angle_rate_imu.data}
    add_3axis_figure(plotter, "pos", pos, fmt='.-')
    add_3axis_figure(plotter, "vel", vel)
    add_3axis_figure(plotter, "acc", acc, linewidth=1)
    add_3axis_figure(plotter, "acc_bias", acc_bias)
    # add_3axis_figure(plotter, "angle_rate", angle_rate)
    plotter.show()

    

