import rospy
import rosbag
import numpy as np
from sys import argv

def derivative(p, t):
    if(len(t) != len(p)):
        print("length mismatch")
        quit()
    d2t = t[2:] - t[:-2]
    dp = p[1:] - p[:-1]
    dt = t[1:] - t[:-1]
    v = 1 / d2t * (dp[1:]*dt[:-1]/dt[1:] + dp[:-1]*dt[1:]/dt[:-1])
    return v

class Sequence4d():
    def __init__(self):
        self.t = np.array([])
        self.x = np.array([])
        self.y = np.array([])
        self.z = np.array([])

    def __init__(self, t, x, y, z):
        self.t = np.array(t)
        self.x = np.array(x)
        self.y = np.array(y)
        self.z = np.array(z)

    def derivative(self):
        dx_dt = derivative(self.x, self.t)
        dy_dt = derivative(self.y, self.t)
        dz_dt = derivative(self.z, self.t)
        return Sequence4d(self.t[1:-1], dx_dt, dy_dt, dz_dt)
    
def Sequence4dFromTransformStamped(msgs):
    t = [msg.header.stamp.to_sec()   for msg in msgs]
    x = [msg.transform.translation.x for msg in msgs]
    y = [msg.transform.translation.y for msg in msgs]
    z = [msg.transform.translation.z for msg in msgs]    
    return Sequence4d(t, x, y, z)

def RotationFromTransformStamped(msgs):
    t = [msg.header.stamp.to_sec()   for msg in msgs]
    r = []
    

def ReadTransformStamped(bag_filename, topic_name):
    msgs = []
    bag = rosbag.Bag(bag_filename)
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        msgs += [msg]
    return msgs

if __name__ == '__main__':
    if(len(argv) < 3):
        print("Example: python acc_from_traj.py bag_path trajectory_topic_name")
        quit()
    bag_filename = argv[1]
    topic_name = argv[2]

    msgs = ReadTransformStamped(bag_filename, topic_name)
    positions = Sequence4dFromTransformStamped(msgs)
    accs = positions.derivative().derivative()
    print(len(positions.t))
    print(len(accs.t))
    

