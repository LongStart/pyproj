import json
import matplotlib.pyplot as plt
from math import *
import sys
import copy

def distance(x0, y0, x1, y1):
    dx = x1 - x0
    dy = y1 - y0
    return (dx*dx + dy*dy)**0.5

if(len(sys.argv) != 3):
    print("Please input json file path!")
    print("Example: python3 cut_road_by_frenet_coordinate.py road_0807.json cutting_rules.json")
    quit()

road_filename = sys.argv[1]
print(road_filename)
file = open(road_filename, 'r')
road_filedata = file.read()
file.close()

rule_filename = sys.argv[2]
file = open(rule_filename, 'r')
rule_filedata = file.read()
file.close()

road_dictdata = json.loads(road_filedata)
rule_dictdata = json.loads(rule_filedata)

rule_idx = 0
s = 0

road_sections = {"version":"2.0.0", "sections":[]}
empty_road_section = {"lanemarkings":0, "segments":[]}
road_section = copy.deepcopy(empty_road_section)

prev_x = road_dictdata["road"][0]["refP"][0]
prev_y = road_dictdata["road"][0]["refP"][1]

for p in road_dictdata["road"]:

    s += distance(prev_x, prev_y, p["refP"][0], p["refP"][1])
    prev_x = p["refP"][0]
    prev_y = p["refP"][1]

    if(rule_dictdata["rules"][rule_idx]["begin"] > s):
        continue

    if(rule_dictdata["rules"][rule_idx]["end"] < s):
        print('finish a section')
        road_sections["sections"] += [road_section]
        road_section = copy.deepcopy(empty_road_section)
        print(empty_road_section)
        rule_idx += 1
        if(rule_idx >= len(rule_dictdata["rules"])):
            break
        continue

    road_section["segments"] += [p]
    road_section["lanemarkings"] = rule_dictdata["rules"][rule_idx]["lanemarkings"]

# print(road_sections)
file = open('output.json','w')
json.dump(road_sections, file, indent=4 )
file.close()