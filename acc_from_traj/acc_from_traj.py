import rospy
import rosbag
import numpy as np
from sys import argv
from offline_tf import OfflineTfBuffer
from tf.transformations import rotation_from_matrix
from tf.transformations import quaternion_matrix
import PlotCollection
# import add_3axis_figure
from add_3axis_figure import *


# def derivative(p, t):
#     assert(len(t) == len(p))
#     d2t = t[2:] - t[:-2]
#     dp = p[1:] - p[:-1]
#     dt = t[1:] - t[:-1]
#     v = 1 / d2t * (dp[1:]*dt[:-1]/dt[1:] + dp[:-1]*dt[1:]/dt[:-1])
#     return v

# def derivative3d(txyz):
#     dx_dt = derivative(txyz[1], txyz[0])
#     dy_dt = derivative(txyz[2], txyz[0])
#     dz_dt = derivative(txyz[3], txyz[0])
#     return np.array([txyz[0][1:-1], dx_dt, dy_dt, dz_dt])

# def Integral3d(txyz):
#     x = np.cumsum(txyz[0]*txyz[1])
#     y = np.cumsum(txyz[0]*txyz[2])
#     z = np.cumsum(txyz[0]*txyz[3])
#     return np.array([txyz[0], x, y, z])

# def MovingAverage(p, t, half_width=3):
#     assert(len(t) == len(p))
#     width = 2*half_width + 1
#     dt = t[1:] - t[0:-1]
#     pp = (p[1:] + p[0:-1])/2
#     pt = pp*dt
#     p_sum = np.convolve(pt, np.ones(width),'valid')
#     t_sum = np.convolve(dt, np.ones(width),'valid')
#     return (p_sum / t_sum)

# def MovingAverage3d(txyz, half_width=3):
#     x = MovingAverage(txyz[1], txyz[0], half_width)
#     y = MovingAverage(txyz[2], txyz[0], half_width)
#     z = MovingAverage(txyz[3], txyz[0], half_width)
#     t_begin = half_width
#     t_end = len(txyz[0]) - half_width -1
#     return np.array([txyz[0][t_begin:t_end], x, y, z])
    
# def PositionMatrixFromTransformStamped(msgs):
#     # txyz = [[msg.header.stamp.to_sec(), msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z] for msg in msgs]
#     t = [msg.header.stamp.to_sec()   for msg in msgs]
#     x = [msg.transform.translation.x for msg in msgs]
#     y = [msg.transform.translation.y for msg in msgs]
#     z = [msg.transform.translation.z for msg in msgs]  
    
#     return np.array([t,x,y,z])

# def SE3FromTransformStamped(msgs):
#     t = [msg.header.stamp.to_sec()   for msg in msgs]
#     x = [msg.transform.translation.x for msg in msgs]
#     y = [msg.transform.translation.y for msg in msgs]
#     z = [msg.transform.translation.z for msg in msgs]  
#     qs = [msg.transform.rotation for msg in msgs]
#     r = [quaternion_matrix([q.x, q.y, q.z, q.w])[0:-1,0:-1] for q in qs]
#     return np.array([t,x,y,z,r])

# def ToBodyFrame(txyzr):
#     txyz = np.array(txyzr[0:-1,:])
#     for i in range(len(txyz[0])):
#         r = txyzr[4][i].transpose()
#         p = np.array(txyz[1:,i])
#         txyz[1:,i] = r.transpose().dot(p)
#     return txyz

def ToBodyFrame(txyz_in, r):
    assert(len(txyz_in.transpose()) == len(r))
    txyz = np.array(txyz_in)
    for i in range(len(txyz[0])):
        p = np.array(txyz[1:,i])
        txyz[1:,i] = r[i].dot(p)
    return txyz

# def RotationFromTransformStamped(msgs):
#     t = [msg.header.stamp.to_sec() for msg in msgs]
#     qs = [msg.pose.orientation for msg in msgs]
#     r = [quaternion_matrix(rotation_from_matrix([q.x, q.y, q.z, q.w])) for q in qs]
#     return (t, r)

# def AccelerationFromIMU(msgs):
#     t = [msg.header.stamp.to_sec() for msg in msgs]
#     x = [msg.linear_acceleration.x for msg in msgs]
#     y = [msg.linear_acceleration.y for msg in msgs]
#     z = [msg.linear_acceleration.z for msg in msgs] 
#     # return np.array([t,x,y,z])
#     return np.array([t,y,z,x])

def GenerateGlobalGravity(t, value=9.81):
    g = np.array([value] * len(t))
    return np.array([t, np.zeros(len(t)), np.zeros(len(t)), g])

def SecondsToRosTime(ts):
    return [rospy.Time(secs=t) for t in ts]
    
def ReadTopicMsg(bag_filename, topic_name):
    msgs = []
    bag = rosbag.Bag(bag_filename)
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        msgs += [msg]
    return msgs

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

    tf_buffer = OfflineTfBuffer(transform_msgs)

    pose_se3 = SE3FromTransformStamped(transform_msgs)
    vels = derivative3d(pose_se3)
    acc_w = derivative3d(vels)
    world_to_body = SE3FromTransformStamped(tf_buffer.LookupTransform(body_frame_name, groundtruth_frame_name, SecondsToRosTime(acc_w[0])))[4]
    acc_b = ToBodyFrame(acc_w, world_to_body)
    ave_acc_b = MovingAverage3d(acc_b, half_width=10)
    
    imu_msgs = ReadTopicMsg(bag_filename, imu_topic_name)
    acc_imu = AccelerationFromIMU(imu_msgs)
    ave_acc_imu = MovingAverage3d(acc_imu, half_width=10)
    # vel_imu = Integral3d(acc_imu)

    #gravity
    # print(np.shape(ave_acc_imu))
    (t_begin, t_end) = tf_buffer.AvailableTimeRange(body_frame_name, groundtruth_frame_name, SecondsToRosTime(ave_acc_imu[0]))
    ave_acc_imu = ave_acc_imu[:, t_begin: t_end]
    g_w = GenerateGlobalGravity(ave_acc_imu[0])
    world_to_body = SE3FromTransformStamped(tf_buffer.LookupTransform(body_frame_name, groundtruth_frame_name, SecondsToRosTime(g_w[0])))[4]
    g_b = ToBodyFrame(g_w, world_to_body)
    print(np.shape(g_b))
    ave_acc_imu[1:] -= g_b[1:] 

    plotter = PlotCollection.PlotCollection("My window name")
    # pos = {'pos_b': positions, 'pos_w': se3[0:4]}
    pos = {'pos_w': pose_se3[0:4]}
    vel = {'vel': vels}
    acc = {
        'ave_acc_b': ave_acc_b, 
        # 'acc_w': acc_w, 
        # 'acc_imu': acc_imu, 
        'ave_acc_imu': ave_acc_imu}
    add_3axis_figure(plotter, "pos", pos)
    add_3axis_figure(plotter, "vel", vel)
    add_3axis_figure(plotter, "acc", acc)
    plotter.show()

    

