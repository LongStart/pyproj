import rospy
import rosbag
import numpy as np
from sys import argv
from vicon_correction import CorrectBiasedStamp
from scipy.spatial.transform import Rotation as R
from ros_io import *
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

    transform_msgs = ReadTopicMsg(bag_filename, groundtruth_topic_name)
    imu_msgs = ReadTopicMsg(bag_filename, imu_topic_name)

    raw_gt_pose = PoseFromTransformStamped(transform_msgs)
    CorrectBiasedStamp(raw_gt_pose[0])

    raw_imu_angle_rate = AngleRateFromIMU(imu_msgs)
    body_angle_rate = AngleRateInBody(raw_gt_pose[0],raw_gt_pose[4:])
    imu_angle_rate = np.array(body_angle_rate)
    imu_angle_rate[1:, :] = StaticRotVec3d(q_vicon_to_imu, body_angle_rate[1:,:])

    print(imu_angle_rate[:,1:5])

    plotter = PlotCollection.PlotCollection("My window name")
    ppp_txyzw = np.array([raw_gt_pose[0], raw_gt_pose[4], raw_gt_pose[5], raw_gt_pose[6], raw_gt_pose[7]])
    
    quat = {'q_from_traj': ppp_txyzw}
    gyro = {
        'angle_rate_from_atraj': body_angle_rate,
        'angle_rate_from_gyro': raw_imu_angle_rate}
    add_3axis_figure(plotter, "angle_rate", gyro, fmt='-')
    add_naxis_figure(plotter, "quat",4,  quat, fmt='.-')
    
    plotter.show()