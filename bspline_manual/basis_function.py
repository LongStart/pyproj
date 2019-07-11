import matplotlib.pyplot as plt
import numpy as np

def basis(degree, knot_vector, i, u):
    if 0 == degree :
        return np.array([float(knot_vector[i] <= v < knot_vector[i + 1]) for v in u])

    coeff_0 = 1.
    if(not np.isclose(knot_vector[i + degree], knot_vector[i])):
        coeff_0 = (u - knot_vector[i])/(knot_vector[i + degree] - knot_vector[i])
    coeff_1 = 1.
    if(not np.isclose(knot_vector[i + degree + 1], knot_vector[i + 1])):
        coeff_1 = (knot_vector[i + degree + 1] - u)/(knot_vector[i + degree + 1] - knot_vector[i + 1])
    return coeff_0 * basis(degree - 1, knot_vector, i, u) + coeff_1 * basis(degree - 1, knot_vector, i + 1, u)

def normalize(v_1d):
    return v_1d / (v_1d.dot(v_1d)**0.5)

def check_knot_vector(knot_vector, step=1.e-8):
    checked_vector = np.array(knot_vector)
    inc = 0.
    for i in range(1, len(checked_vector)):
        if checked_vector[i] + inc < checked_vector[i - 1]:
            raise RuntimeError('knot vector not increasing')
        if np.isclose(checked_vector[i] + inc, checked_vector[i - 1]):
            inc += step
            checked_vector[i] += inc
    return checked_vector
            


class bspline(object):
    def __init__(self, degree, knot_vector, control_point):
        assert(len(knot_vector) == degree + len(control_point) + 1)
        self.degree = (degree)
        self.knot_vector = check_knot_vector(knot_vector)
        self.control_point = control_point
        

    def __call__(self, x):
        sum_y = np.zeros(len(x))
        for i in range(len(self.control_point)):
            y = (basis(self.degree, self.knot_vector, i, x))*self.control_point[i]
            sum_y += y
        return sum_y

    def curve(resolution=50):
        t = np.linspace(self.knot_vector[2], self.knot_vector[-3], resolution)
        sum_y = np.zeros(len(t))
        for i in range(len(self.control_point)):
            y = (basis(self.degree, self.knot_vector, i, t))*self.control_point[i]
            sum_y += y
        return (t, sum_y)

    def basis_i(self, i, x):
        return (basis(self.degree, self.knot_vector, i, x))*self.control_point[i]


if __name__ == "__main__":
    # degree = 2
    # knot_vector = np.array([3,3,3,4.,5.,6.,7,7,7])
    # control_point_x = (np.array([1.,1,1,1,1,1]))
    # control_point_y = (np.array([1.,5,6,5,1,2]))
    degree = 3
    # knot_vector = np.array([4,4,4,4,5,6,6,6,6])
    knot_vector = np.array([0,1,2,3,4,5,6,7,8])
    control_point_x = (np.array([-2,2,1,7,3]))
    control_point_y = (np.array([1,5,6,5,2]))
    resolution = 200
    bx = bspline(degree, knot_vector, control_point_x)
    by = bspline(degree, knot_vector, control_point_y)
    t = np.linspace(bx.knot_vector[2], bx.knot_vector[-3], resolution, endpoint=False)
    # print(t)
    
    xs = bx(t)
    ys = by(t)
    plt.figure()
    plt.grid(1)
    plt.plot(t, xs)
    for i in range(len(control_point_x)):
        plt.plot(t, bx.basis_i(i, t))
    plt.figure()
    plt.grid(1)
    plt.plot(t, ys)
    for i in range(len(control_point_y)):
        plt.plot(t, by.basis_i(i, t))
    plt.figure()
    plt.plot(control_point_x, control_point_y, 'o-')
    plt.plot(xs, ys)
    plt.grid(1)
    plt.show()