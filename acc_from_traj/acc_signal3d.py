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


def ToBodyFrame(txyz_in, r):
    assert(len(txyz_in.transpose()) == len(r))
    txyz = np.array(txyz_in)
    for i in range(len(txyz[0])):
        p = np.array(txyz[1:,i])
        txyz[1:,i] = r[i].dot(p)
    return txyz


def GenerateGlobalGravity(t, value=9.81):
    g = np.array([value] * len(t))
    return np.array([t, np.zeros(len(t)), np.zeros(len(t)), g])

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

    pos_world = Signal3d(PositionFromTransformStamped(transform_msgs))
    vel_world = pos_world.Derivative()
    acc_world = vel_world.Derivative()

    body_to_world = Signal3d(RotationFromTransformStamped(transform_msgs))
    world_to_body = body_to_world*(-1)

    acc_body = acc_world.Rotate(world_to_body)
    ave_acc_body = acc_body.MovingAverage(half_width=10)
    
    imu_msgs = ReadTopicMsg(bag_filename, imu_topic_name)
    acc_imu = Signal3d(AccelerationFromIMU(imu_msgs))
    ave_acc_imu = acc_imu.MovingAverage(half_width=10)

    #gravity
    g_world = Signal3d(GenerateGlobalGravity(ave_acc_imu.t()))
    g_body = g_world.Rotate(world_to_body)
    ave_acc_imu = ave_acc_imu - g_body
    acc_bias = ave_acc_body - ave_acc_imu 

    plotter = PlotCollection.PlotCollection("My window name")
    pos = {'pos_w': pos_world.data}
    vel = {'vel': vel_world.data}
    acc = {
        'ave_acc_b': ave_acc_body.data, 
        'ave_acc_imu': ave_acc_imu.data}
    acc_bias = {'acc_bias': acc_bias.data}
    add_3axis_figure(plotter, "pos", pos)
    add_3axis_figure(plotter, "vel", vel)
    add_3axis_figure(plotter, "acc", acc)
    add_3axis_figure(plotter, "acc_bias", acc_bias)
    plotter.show()

    

