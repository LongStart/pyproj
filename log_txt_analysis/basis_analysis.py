import sys
import numpy as np

def LoadLogTxt(filename):
    data_matrix = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            input_data = line.split(" ")
            if input_data[0] != "IMG":
                continue
            data_matrix.append([float(input_data[1]), float(input_data[-1])])
    return np.array(data_matrix).T

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("example: python {} <log.txt>".format(sys.argv[0]))
        quit()

    input_filename = sys.argv[1]
    data_matrix = LoadLogTxt(input_filename)
    # print(data_matrix)
    dt = data_matrix[0, 1:] - data_matrix[0, :-1]
    print(dt)
    fps = (len(dt)) / np.sum(dt) * 1e9
    print(fps)