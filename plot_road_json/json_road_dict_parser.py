from math import *

def parser_1_0_3(dictdata):
    x = []
    y = []

    r_x = []
    r_y = []

    remove_offset = False

    x_start = float(dictdata["road"][0]['refP'][0])
    y_start = float(dictdata["road"][0]['refP'][1])

    if False == remove_offset :
        x_start = 0
        y_start = 0

    for p in dictdata["road"]:
        x0 = float(p['refP'][0])
        y0 = float(p['refP'][1])
        r_x += [x0 - x_start]
        r_y += [y0 - y_start]

        heading = float(p['heading'])
        
        x_temp = x0
        y_temp = y0
        for w in p["widths"]:
            x_temp += w[1]*sin(heading)
            y_temp += -w[1]*cos(heading)
            x += [x_temp - x_start]
            y += [y_temp - y_start]

    return (r_x,r_y,x,y)

def parser_1_0_4(dictdata):
    x = []
    y = []

    r_x = []
    r_y = []

    remove_offset = False

    x_start = float(dictdata["road"][0]['refP'][0])
    y_start = float(dictdata["road"][0]['refP'][1])

    if False == remove_offset :
        x_start = 0
        y_start = 0

    for p in dictdata["road"]:
        x0 = float(p['refP'][0])
        y0 = float(p['refP'][1])
        r_x += [x0 - x_start]
        r_y += [y0 - y_start]

        heading = float(p['heading'])
        
        for w in p["widths"]:
            x_temp = x0 + w[1]*sin(heading)
            y_temp = y0 -w[1]*cos(heading)
            x += [x_temp - x_start]
            y += [y_temp - y_start]
    
    return (r_x,r_y,x,y)

