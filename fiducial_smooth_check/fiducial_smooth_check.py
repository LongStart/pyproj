import rosbag
from math import * 
import matplotlib.pyplot as plt

def norm(vec3):
    return (vec3.x*vec3.x + vec3.y*vec3.y + vec3.z*vec3.z)**0.5

def angle_from_w(w):
    return 2*acos(w)*180/pi

if __name__ == '__main__':
    from sys import argv
    if(len(argv) < 2):
        print("Example: python xxx.py bag_path")
        quit()
    bag_filename = argv[1]
    bag = rosbag.Bag(bag_filename)
    marker_infos = {}
    for topic, msg, t in bag.read_messages(topics=['/fiducial_transforms']):
        img_seq = msg.image_seq
        for info in msg.transforms:
            tag_id = info.fiducial_id
            if not tag_id in marker_infos:
               marker_infos[tag_id] = []
            marker_infos[tag_id] += [[img_seq, norm(info.transform.translation), angle_from_w(info.transform.rotation.w)]]

    for key in marker_infos:
        (seq, trans, theta) = zip(*marker_infos[key])
        # plt.plot(seq, trans, '.-', label=str(key))
        plt.plot(seq, theta, '.-', label=str(key))

    plt.grid(1)
    plt.legend()
    plt.show()