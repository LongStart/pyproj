import numpy as np
import re
import matplotlib.pyplot as plt
from sys import argv

def MatchListToArray(match_list):
    data = []
    for match in match_list:
        data.append([float(s) for s in match])
    return np.vstack(data).transpose()

def ExtractIMUData(filename):
    gyro_regex_pattern = 'GYRO\s(.*?)\s(.*?)\s(.*?)\s(.*?)\n'
    accel_regex_pattern = 'ACCEL\s(.*?)\s(.*?)\s(.*?)\s(.*?)\n'

    log_file = open(filename, 'r')
    file_text = log_file.read()

    match_list = re.findall(gyro_regex_pattern, file_text)
    gyro_txyz = MatchListToArray(match_list)

    match_list = re.findall(accel_regex_pattern, file_text)
    accel_txyz = MatchListToArray(match_list)
    return gyro_txyz, accel_txyz

if __name__ == "__main__":
    if(len(argv) < 2):
        print("example: python imu_time_intarval.py home/abc/log.txt home/bcd/log.txt")
        quit()

    for i in range(1, len(argv)):
        gyro_txyz, accel_txyz = ExtractIMUData(argv[i])
        gyro_dt = gyro_txyz[0, 1:] - gyro_txyz[0, :-1]
        gyro_dt /= 1e6
        # accel_dt = accel_txyz[0, 1:] - accel_txyz[0, :-1]
        plt.plot(gyro_dt, '_', label=argv[i].split('/')[-2])

    plt.legend()
    plt.grid(1)
    plt.ylabel('delta time(ms)')
    plt.xlabel('timestampe(s)')
    plt.show()
    
