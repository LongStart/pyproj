import rospy
import rosbag
import numpy as np
from sys import argv
from tf.transformations import rotation_from_matrix
from tf.transformations import quaternion_matrix
import PlotCollection
# import add_3axis_figure
from add_3axis_figure import *


def derivative(p, t):
    if(len(t) != len(p)):
        print("length mismatch")
        quit()
    d2t = t[2:] - t[:-2]
    dp = p[1:] - p[:-1]
    dt = t[1:] - t[:-1]
    v = 1 / d2t * (dp[1:]*dt[:-1]/dt[1:] + dp[:-1]*dt[1:]/dt[:-1])
    return v

def derivative3d(txyz):
    dx_dt = derivative(txyz[1], txyz[0])
    dy_dt = derivative(txyz[2], txyz[0])
    dz_dt = derivative(txyz[3], txyz[0])
    return np.array([txyz[0][1:-1], dx_dt, dy_dt, dz_dt])

def PositionMatrixFromTransformStamped(msgs):
    # txyz = [[msg.header.stamp.to_sec(), msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z] for msg in msgs]
    t = [msg.header.stamp.to_sec()   for msg in msgs]
    x = [msg.transform.translation.x for msg in msgs]
    y = [msg.transform.translation.y for msg in msgs]
    z = [msg.transform.translation.z for msg in msgs]  
    
    return np.array([t,x,y,z])

def SE3FromTransformStamped(msgs):
    t = [msg.header.stamp.to_sec()   for msg in msgs]
    x = [msg.transform.translation.x for msg in msgs]
    y = [msg.transform.translation.y for msg in msgs]
    z = [msg.transform.translation.z for msg in msgs]  
    qs = [msg.transform.rotation for msg in msgs]
    r = [quaternion_matrix([q.x, q.y, q.z, q.w])[0:-1,0:-1] for q in qs]
    return np.array([t,x,y,z,r])

def ToBodyFrame(txyzr):
    txyz = np.array(txyzr[0:-1,:])
    for i in range(len(txyz[0])):
        r = txyzr[4][i].transpose()
        p = np.array(txyz[1:,i])
        txyz[1:,i] = r.transpose().dot(p)
    return txyz

def RotationFromTransformStamped(msgs):
    t = [msg.header.stamp.to_sec() for msg in msgs]
    qs = [msg.pose.orientation for msg in msgs]
    r = [quaternion_matrix(rotation_from_matrix([q.x, q.y, q.z, q.w])) for q in qs]
    return (t, r)

def AccelerationFromIMU(msgs):
    t = [msg.header.stamp.to_sec() for msg in msgs]
    x = [msg.linear_acceleration.x for msg in msgs]
    y = [msg.linear_acceleration.y for msg in msgs]
    z = [msg.linear_acceleration.z for msg in msgs] 
    return np.array([t,x,y,z])
    
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
    else:
        print('unknown dataset')
        quit()

    transform_msgs = ReadTopicMsg(bag_filename, groundtruth_topic_name)
    se3 = SE3FromTransformStamped(transform_msgs)
    positions = ToBodyFrame(se3)
    vels = derivative3d(positions)
    accs = derivative3d(vels)
    
    imu_msgs = ReadTopicMsg(bag_filename, imu_topic_name)
    acc_imu = AccelerationFromIMU(imu_msgs)

    plotter = PlotCollection.PlotCollection("My window name")
    pos = {'pos_b': positions, 'pos_w': se3[0:4]}
    vel = {'vel': vels}
    acc = {'acc': accs, 'acc_imu': acc_imu}
    add_3axis_figure(plotter, "pos", pos)
    add_3axis_figure(plotter, "vel", vel)
    add_3axis_figure(plotter, "acc", acc)
    plotter.show()

    

