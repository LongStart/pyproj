import json
import sys

if(len(sys.argv) != 3):
    print("Please input json file path!")
    print("Example: python3 converto_to_gmm_input.py /home/ad/chenxx/road_json/road_0807.json output.json")
    quit()

filename = sys.argv[1]
print(filename)
file = open(filename, 'r')
filedata = file.read()
file.close()

dictdata = json.loads(filedata)

points = []
for section in dictdata["road"]:
    for width in section["widths"]:
        if(width[1] != 0):
            points += [[width[1]]]

point_dict = {"points":points}

output = open(sys.argv[2], 'w')
json.dump(point_dict, output, indent=4)
output.close()