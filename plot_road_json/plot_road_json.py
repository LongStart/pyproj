import json
import matplotlib.pyplot as plt
from math import *
import sys
from json_road_dict_parser import *

if(len(sys.argv) != 2):
    print("Please input json file path!")
    print("Example: python3 plot_road_json.py /home/ad/chenxx/road_json/road_0807.json")
    quit()

filename = sys.argv[1]
print(filename)
file = open(filename, 'r')
filedata = file.read()
file.close()

dictdata = json.loads(filedata)


parser_lib = {
    "1.0.1":parser_1_0_2,
    "1.0.2":parser_1_0_2,
    "1.0.3":parser_1_0_3}

try:
    (r_x, r_y, x, y) = parser_lib[dictdata["version"]](dictdata)
except KeyError:
    print('unsupport file version: ' + dictdata["version"])
    quit()

print('json road version: ' + dictdata["version"])        

plt.plot(x,y,'.')
plt.plot(r_x,r_y,'o',markersize=1.5)
plt.axis('equal')
plt.show()



