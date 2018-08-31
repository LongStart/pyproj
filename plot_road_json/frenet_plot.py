import json
import matplotlib.pyplot as plt
from math import *
import sys

def distance(x0, y0, x1, y1):
    dx = x1 - x0
    dy = y1 - y0
    return (dx*dx + dy*dy)**0.5

def frenet_parser_1_0_3(dictdata):
    x = []
    y = []

    r_x = []
    r_y = []

    s = 0
    prev_ref_x = dictdata["segments"][0]['refPoint']["x"]
    prev_ref_y = dictdata["segments"][0]['refPoint']["y"]
    
    count = 0
    for p in dictdata["segments"]:
        curr_ref_x = p['refPoint']['x']
        curr_ref_y = p['refPoint']['y']

        s += distance(prev_ref_x, prev_ref_y, curr_ref_x, curr_ref_y)

        curr_ref_x_frenet = s
        curr_ref_y_frenet = 0

        r_x += [curr_ref_x_frenet]
        r_y += [curr_ref_y_frenet]

        prev_ref_x = curr_ref_x
        prev_ref_y = curr_ref_y

        # print('curr_ref_x_frenet: ' + str(curr_ref_x_frenet) + ', idx: ' + str(count))
        # count += 1
        # input()    
        for w in p["offsets"]:
            x_temp = curr_ref_x_frenet
            y_temp = curr_ref_y_frenet - w['offset']
            x += [x_temp]
            y += [y_temp]
    
    return (r_x,r_y,x,y)

if(len(sys.argv) != 2):
    print("Please input json file path!")
    print("Example: python3 frenet_plot.py /home/ad/chenxx/road_json/road_0807.json")
    quit()

filename = sys.argv[1]
print(filename)
file = open(filename, 'r')
filedata = file.read()
file.close()

dictdata = json.loads(filedata)

(r_x, r_y, x, y) = frenet_parser_1_0_3(dictdata)

# print('json road version: ' + dictdata["version"])        

plt.plot(x,y,'.')
plt.plot(r_x,r_y,'o',markersize=1.5)
plt.axis('equal')
plt.show()