if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt

    control_points = np.array([[0,0],[1,3],[4,4],[6,8]])
    x = control_points[:,0]
    y = control_points[:,1]
    t = np.linspace(0,1,100)
    px = (1-t)*(1-t)*(1-t)*x[0]+3*t*(1-t)*(1-t)*x[1]+3*t*t*(1-t)*x[2]+t*t*t*x[3]
    py = (1-t)*(1-t)*(1-t)*y[0]+3*t*(1-t)*(1-t)*y[1]+3*t*t*(1-t)*y[2]+t*t*t*y[3]
    plt.plot(px, py)
    plt.plot(x,y,'x')
    plt.show()
    