from math import *
def loc_file_to_list(file_name, rotate=0.0):
    f = open(file_name)
    loc_x = []
    loc_y = []

    first_line = f.readline().split()

    loc_x_off = float(first_line[0])
    loc_y_off = float(first_line[1])

    for line in f:
        x = float(line.split()[0]) - loc_x_off
        y = float(line.split()[1]) - loc_y_off
        loc_x += [x*cos(rotate) - y*sin(rotate)]
        loc_y += [x*sin(rotate) + y*cos(rotate)]
    
    return (loc_x, loc_y)
