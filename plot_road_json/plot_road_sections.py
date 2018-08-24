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

for section in dictdata["sections"]:
    (r_x,r_y,x,y) = parser_1_0_3(section,"segments")
    plt.plot(x,y,'.')
    # plt.plot(r_x,r_y,'o',markersize=1.5)

plt.axis('equal')
plt.show()



