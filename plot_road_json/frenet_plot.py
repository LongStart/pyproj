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
    prev_r_x = float(dictdata["road"][0]['refP'][0])
    prev_r_y = float(dictdata["road"][0]['refP'][1])
    
    for p in dictdata["road"]:
        x0 = s
        y0 = 0

        s += distance(prev_r_x, prev_r_y, float(p['refP'][0]), float(p['refP'][1]))

        r_x += [x0]
        r_y += [y0]

        prev_r_x = p['refP'][0]
        prev_r_y = p['refP'][1]
        
        for w in p["widths"]:
            x_temp = x0
            y_temp = y0 - w[1]
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

print('json road version: ' + dictdata["version"])        

plt.plot(x,y,'.')
plt.plot(r_x,r_y,'o',markersize=1.5)
plt.axis('equal')
plt.show()