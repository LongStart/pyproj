import json
import matplotlib.pyplot as plt
from math import *

filename = '/home/ad/chenxx/road_json/road_0807.json'
file = open(filename, 'r')
filedata = file.read()
file.close()

dictdata = json.loads(filedata)

x = []
y = []

r_x = []
r_y = []

x_start = float(dictdata["road"][0]['refP'][0])
y_start = float(dictdata["road"][0]['refP'][1])

for p in dictdata["road"]:
    x0 = float(p['refP'][0])
    y0 = float(p['refP'][1])
    r_x += [x0 - x_start]
    r_y += [y0 - y_start]

    heading = float(p['heading'])
    
    x_temp = x0
    y_temp = y0
    for w in p["widths"]:
        x_temp += -w[1]*sin(heading)
        y_temp += w[1]*cos(heading)
        x += [x_temp - x_start]
        y += [y_temp - y_start]
        

plt.plot(x,y,'.')
plt.plot(r_x,r_y,'.')
plt.axis('equal')
plt.show()



