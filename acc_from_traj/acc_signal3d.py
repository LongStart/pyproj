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

def RotVecBetweenVector3d(v1, v2):
    cross = np.cross(v1, v2)
    cross_norm = np.linalg.norm(cross)
    dot = v1.dot(v2)
    return cross* np.arctan2(cross_norm, dot) /cross_norm

if __name__ == '__maindd__':
    g_imu = np.array([9.34, 0.32, -3.22])
    g_world = np.array([0, 0, 9.8])
    rotv = RotationBetweenVector3d(g_imu, g_world)
    r = R.from_rotvec(rotv)
    print(r.apply(g_imu))

if __name__ == '__main__':
    if(len(argv) < 3):
        print("Example: python acc_from_traj.py dataset_name bag_path ")
        quit()
    dataset_name = argv[1]
    bag_filename = argv[2]
    
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

    raw_gt_position = PositionFromTransformStamped(transform_msgs)
    CorrectBiasedStamp(raw_gt_position[0])

    pos_gt_world = Signal3d(raw_gt_position)
    vel_gt_world = pos_gt_world.Derivative()
    acc_gt_world = vel_gt_world.Derivative()

    
    # quit()

    body_to_world = Signal3d(RotationFromTransformStamped(transform_msgs))
    world_to_body = body_to_world*(-1)

    acc_gt_body = acc_gt_world.Rotate(world_to_body)
    # ave_acc_gt_body = acc_gt_body.LFilter(0.06)
    ave_acc_gt_body = acc_gt_body.MovingAverage(50)
    
    imu_msgs = ReadTopicMsg(bag_filename, imu_topic_name)
    acc_sensor_imu = Signal3d(AccelerationFromIMU(imu_msgs))
    g_vec_world = np.array([0, 0, 9.8])
    ave_num = 100
    g_t0_imu = sum(acc_sensor_imu.xyz().transpose()[0:ave_num+1])/ave_num
    
    imu_to_body = Signal3d.from_vector(RotVecBetweenVector3d(g_t0_imu, g_vec_world))
    body_to_imu = imu_to_body * (-1)
    
    acc_sensor_body = acc_sensor_imu.Rotate(imu_to_body)
    g_world = Signal3d.from_vector(g_vec_world, t=acc_sensor_imu.t())
    g_body = g_world.Rotate(world_to_body)  
    
    acc_sensor_body = acc_sensor_body - g_body
    
    # ave_acc_sensor_body = acc_sensor_body.LFilter(0.02)
    ave_acc_sensor_body = acc_sensor_body.MovingAverage(100)
    acc_bias_body = ave_acc_sensor_body - ave_acc_gt_body 

    plotter = PlotCollection.PlotCollection("My window name")
    pos = {'pos_gt_world': pos_gt_world.data}
    vel = {'vel_gt_world': vel_gt_world.data}
    acc = {
        'ave_acc_gt_body': ave_acc_gt_body.data,
        'ave_acc_sensor_body': ave_acc_sensor_body.data, 
        }
    acc_bias = {'acc_bias_body': acc_bias_body.data}
    # angle_rate = {'angle_rate_imu': angle_rate_imu.data}
    add_3axis_figure(plotter, "pos", pos, fmt='.-')
    add_3axis_figure(plotter, "vel", vel)
    add_3axis_figure(plotter, "acc", acc, linewidth=1)
    add_3axis_figure(plotter, "acc_bias", acc_bias)
    # add_3axis_figure(plotter, "angle_rate", angle_rate)
    plotter.show()

    

