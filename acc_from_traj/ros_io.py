import rospy
import rosbag
from scipy.spatial.transform import Rotation as R
import numpy as np

def PositionFromTransformStamped(msgs):
    t = [msg.header.stamp.to_sec()   for msg in msgs]
    x = [msg.transform.translation.x for msg in msgs]
    y = [msg.transform.translation.y for msg in msgs]
    z = [msg.transform.translation.z for msg in msgs]  
    return np.array([t,x,y,z])

def RotationFromTransformStamped(msgs):
    t = [msg.header.stamp.to_sec() for msg in msgs]
    qs = [msg.transform.rotation for msg in msgs]
    r = np.array([R.from_quat([q.x, q.y, q.z, q.w]).as_rotvec() for q in qs]).transpose()
    return np.array([t, r[0], r[1], r[2]])

def AccelerationFromIMU(msgs):
    t = [msg.header.stamp.to_sec() for msg in msgs]
    x = [msg.linear_acceleration.x for msg in msgs]
    y = [msg.linear_acceleration.y for msg in msgs]
    z = [msg.linear_acceleration.z for msg in msgs] 
    return np.array([t,x,y,z])
#     return np.array([t,y,z,x])
        

def AngleRateFromIMU(msgs):
    t = [msg.header.stamp.to_sec() for msg in msgs]
    x = [msg.angular_velocity.x for msg in msgs]
    y = [msg.angular_velocity.y for msg in msgs]
    z = [msg.angular_velocity.z for msg in msgs] 
    return np.array([t,y,z,x])

def SecondsToRosTime(ts):
    return [rospy.Time(secs=t) for t in ts]
    
def ReadTopicMsg(bag_filename, topic_name, limit=0):
    msgs = []
    bag = rosbag.Bag(bag_filename)
    count = 0
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        msgs += [msg]
        count += 1
        if limit > 0 and count > limit:
            break
    return msgs