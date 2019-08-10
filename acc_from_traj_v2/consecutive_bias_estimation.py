import rospy
import rosbag
import numpy as np
from sys import argv
from vicon_correction import CorrectBiasedStamp
from scipy.spatial.transform import Rotation as R
from scipy.ndimage.filters import uniform_filter1d
from ros_io import *
from inertia_from_traj import *
from core.assignable_space_signal import *
import PlotCollection
from add_3axis_figure import *
from core.dsp import *
from consecutive_inertia_from_traj import *

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

    q_vicon_to_imu = R.from_dcm([[0.33638, -0.01749,  0.94156],[-0.02078, -0.99972, -0.01114],[0.94150, -0.01582, -0.33665]]).as_quat()
    t_vicon_to_imu = np.array([0.06901, -0.02781, -0.12395])
    vicon_to_imu_xyz_xyzw = np.hstack((t_vicon_to_imu, q_vicon_to_imu))
    imu_to_vicon_xyz_xyzw = np.hstack((-1 * t_vicon_to_imu, -1 * q_vicon_to_imu))
    gravity = np.array([0., 0., 9.81])

    transform_msgs = ReadTopicMsg(bag_filename, groundtruth_topic_name)
    imu_msgs = ReadTopicMsg(bag_filename, imu_topic_name)

    raw_gt_pose = PoseFromTransformStamped(transform_msgs)
    CorrectBiasedStamp(raw_gt_pose[0])
    raw_gt_pose = Trajectory3d(raw_gt_pose)
    raw_gt_pose.xyzw = ContiguousQuaternion(raw_gt_pose.xyzw) 
    
    raw_imu_angle_rate = Signal3d(AngleRateFromIMU(imu_msgs))
    raw_imu_acc = Signal3d(AccelerationFromIMU(imu_msgs))

    (imu_angle_rate_f, imu_acc_f) = InertiaFuncFromTrajectory(raw_gt_pose.t_xyz_xyzw, vicon_to_imu_xyz_xyzw, gravity)
    resample_t = np.linspace(raw_gt_pose.t[6], raw_gt_pose.t[-6], 20000)
    imu_angle_rate = Signal3d.from_t_xyz(resample_t, imu_angle_rate_f(resample_t))
    imu_acc = Signal3d.from_t_xyz(resample_t, imu_acc_f(resample_t))
    

    plotter = PlotCollection.PlotCollection("Multiple Wave")
    # q_viz_txyzw = np.vstack([raw_gt_pose.t(), ContiguousQuaternion(raw_gt_pose.xyzw())])

    # body_angle_rate_from_imu = np.vstack((raw_imu_angle_rate[0], StaticTransformVec3d(imu_to_vicon_xyz_xyzw, raw_imu_angle_rate[1:])))
    body_angle_rate_from_imu = Signal3d.from_t_xyz(
        raw_imu_angle_rate.t, StaticTransformVec3d(imu_to_vicon_xyz_xyzw, raw_imu_angle_rate.xyz))
    
    quat = {'q_from_traj': raw_gt_pose.t_xyzw}
    gyro = {
        'from_traj': imu_angle_rate.t_xyz,
        'from_imu': raw_imu_angle_rate.t_xyz}
    acc = {
        'from_traj': imu_acc.t_xyz,
        'from_imu': raw_imu_acc.t_xyz}
    # print(gyro.keys()[0])
    # print(acc.keys()[0])
    add_naxis_figure(plotter, "angle_rate", gyro, linewidth=0.8, fmt='-')
    add_naxis_figure(plotter, "quat", quat, fmt='-')
    add_naxis_figure(plotter, "acc", acc, linewidth=0.8, fmt='-')
    
    plotter.show()