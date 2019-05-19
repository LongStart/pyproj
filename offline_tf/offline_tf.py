import rospy
from tf2_ros import Buffer
from sys import argv
import rosbag

class OfflineTfBuffer():
    def __init__(self, transforms):
        self.t_front = transforms[0].header.stamp
        self.t_back = transforms[-1].header.stamp
        self.buff = Buffer(cache_time=(self.t_back - self.t_front))
        for transform in transforms:
            self.buff.set_transform(transform, '/auth')
        

    def LookupTransform(self, target_frame, source_frame, times):
        return [self.buff.lookup_transform(target_frame, source_frame, t) for t in times]

    def AvailableTimeRange(self, target_frame, source_frame, times):
        if times[0] > self.t_back or times[-1] < self.t_front:
            return [] 
        idx_front = 0
        idx_back = len(times) -1
        while times[idx_front] < self.t_front:
            idx_front += 1
        while times[idx_back] > self.t_back:
            idx_back -= 1
        return (idx_front, idx_back+1)
        
    def CanTransform(self, target_frame, source_frame, times):
        return self.buff.can_transform(target_frame, source_frame, times[0]) and self.buff.can_transform(target_frame, source_frame, times[-1])

if __name__ == '__main__':
    if(len(argv) < 2):
        print("Example: python acc_from_traj.py euroc_bag_path")
        quit()
    bag_filename = argv[1]
    groundtruth_topic_name = '/vicon/firefly_sbx/firefly_sbx'
    world_frame_name = "world"
    body_frame_name = "vicon/firefly_sbx/firefly_sbx"

    def ReadTopicMsg(bag_filename, topic_name):
        msgs = []
        bag = rosbag.Bag(bag_filename)
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            msgs += [msg]
        return msgs

    transform_msgs = ReadTopicMsg(bag_filename, groundtruth_topic_name)
    tf_buffer = OfflineTfBuffer(transform_msgs)
    ts = [msg.header.stamp for msg in transform_msgs]
    print(tf_buffer.CanTransform(world_frame_name, body_frame_name, ts[1:20]))
    results = tf_buffer.LookupTransform(world_frame_name, body_frame_name, ts[1:20])
    # print(tf_buffer.AvailableTimeRange(world_frame_name, body_frame_name, rospy.Time(secs=1000)))
    