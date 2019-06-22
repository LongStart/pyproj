import rospy
import rosbag
import numpy as np
from sys import argv
from vicon_correction import CorrectBiasedStamp
from scipy.spatial.transform import Rotation as R
from ros_io import *
from inertia_from_traj import *
# from signal import Trajectory3d
import PlotCollection
from add_3axis_figure import *
from dsp import *

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
    # rotvec_vicon_to_imu = R.from_quat(q_vicon_to_imu).as_rotvec()
    # print('vicon to imu: angle: {}'.format(rotvec_vicon_to_imu.dot(rotvec_vicon_to_imu)**0.5))
    # quit()
    t_vicon_to_imu = np.array([0.06901, -0.02781, -0.12395])
    vicon_to_imu_xyz_xyzw = np.hstack((t_vicon_to_imu, q_vicon_to_imu))
    gravity = np.array([0., 0., 9.81])

    transform_msgs = ReadTopicMsg(bag_filename, groundtruth_topic_name)
    imu_msgs = ReadTopicMsg(bag_filename, imu_topic_name)

    raw_gt_pose = PoseFromTransformStamped(transform_msgs)
    CorrectBiasedStamp(raw_gt_pose[0])

    raw_imu_angle_rate = AngleRateFromIMU(imu_msgs)
    raw_imu_acc = AccelerationFromIMU(imu_msgs)
    
    (imu_angle_rate, imu_acc_from_traj) = InertiaFromTrajectory(raw_gt_pose, vicon_to_imu_xyz_xyzw, gravity)

    plotter = PlotCollection.PlotCollection("Multiple Wave")
    q_viz_txyzw = np.vstack([raw_gt_pose[0], raw_gt_pose[4:]])
    
    quat = {'q_from_traj': q_viz_txyzw}
    gyro = {
        # 'angle_rate_from_atraj': body_angle_rate,
        'angle_rate_from_btraj': imu_angle_rate,
        'angle_rate_from_gyro': raw_imu_angle_rate}
    acc = {
        'acc_from_traj': imu_acc_from_traj,
        'acc_from_accel': raw_imu_acc}
    add_3axis_figure(plotter, "angle_rate", gyro, linewidth=0.4, fmt='-')
    add_naxis_figure(plotter, "quat", quat, fmt='.-')
    add_naxis_figure(plotter, "acc", acc, linewidth=0.3, fmt='-')
    
    plotter.show()