import matplotlib.pyplot as plt
import numpy as np
# from scipy.interpolate import BSpline
from scipy import interpolate

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
    def __init__(self, degree, knot_vector, control_points):
        assert(len(knot_vector) == degree + len(control_points) + 1)
        self.degree = degree
        self.knot_vector = check_knot_vector(knot_vector)
        self.control_points = control_points
        self.min_x = self.knot_vector[self.degree]
        self.max_x = self.knot_vector[-1-self.degree]
        

    def __call__(self, x):
        sum_y = np.zeros(len(x))
        for i in range(len(self.control_points)):
            y = (basis(self.degree, self.knot_vector, i, x))*self.control_points[i]
            sum_y += y
        return sum_y

    def cum_f(self, x):
        cum_basis = np.zeros(len(x))
        delta_ctrl_points = np.hstack([self.control_points[0], self.control_points[1:] - self.control_points[:-1]])
        sum_y = np.zeros(len(x))
        for i in range(len(self.control_points) - 1, -1, -1):
            cum_basis += self.basis(i, x)
            sum_y += cum_basis * delta_ctrl_points[i]
        return sum_y

    def curve(self, resolution=50):
        t = np.linspace(self.knot_vector[self.degree], self.knot_vector[-1-self.degree], resolution)
        sum_y = np.zeros(len(t))
        for i in range(len(self.control_points)):
            y = (basis(self.degree, self.knot_vector, i, t))*self.control_points[i]
            sum_y += y
        return (t, sum_y)

    def basis(self, i, x):
        return (basis(self.degree, self.knot_vector, i, x))

    def weighted_basis(self, i, x):
        return (self.basis(i, x))*self.control_points[i]


def deg_viz(deg):
    knot_vector = np.array([0.,1,2,3,4,5,6,7,8,9,10,11,12,13,14])
    control_points = np.array([1.]*(len(knot_vector) - deg - 1))
    bsp = bspline(deg, knot_vector, control_points)

    t = np.linspace(bsp.min_x, bsp.max_x, 50)
    for i in range(len(control_points)):
        plt.plot(t, bsp.basis(i, t))
        # plt.plot(t, bsp.cum_basis(i, t))
    # plt.show()

if __name__ == "__main1__":
    # deg_viz(0)
    # deg_viz(1)
    # deg_viz(2)
    deg_viz(3)
    # deg_viz(4)
    # deg_viz(5)
    plt.show()


if __name__ == "__main__":
    # degree = 2
    # knot_vector = np.array([3,3,3,4.,5.,6.,7,7,7])
    # control_points_x = (np.array([1.,1,1,1,1,1]))
    # control_points_y = (np.array([1.,5,6,5,1,2]))
    degree = 3
    knot_vector = np.array([4,4,4,4,5,6,6,6,6])
    # knot_vector = np.array([3,3,3,4,5,6,7,7,7])
    # knot_vector = np.array([0,0,3,4,5,6,7,10,10])
    # knot_vector = np.array([0,1,2,3,4,5,6,7,8])
    control_points_x = (np.array([-2,2,1,7,3]))
    control_points_y = (np.array([1,5,6,5,2]))
    resolution = 200
    bx = bspline(degree, knot_vector, control_points_x)
    by = bspline(degree, knot_vector, control_points_y)
    t = np.linspace(bx.knot_vector[3], bx.knot_vector[-4], resolution, endpoint=False)

    sci_xs = interpolate.BSpline(knot_vector, control_points_x, degree, extrapolate=False)(t)
    sci_ys = interpolate.BSpline(knot_vector, control_points_y, degree, extrapolate=False)(t)
    # print(t)
    
    xs = bx.cum_f(t)
    ys = by.cum_f(t)
    # plt.figure()
    # plt.grid(1)
    # plt.plot(t, xs)
    # for i in range(len(control_points_x)):
    #     plt.plot(t, bx.basis_i(i, t))
    # plt.figure()
    # plt.grid(1)
    # plt.plot(t, ys)
    # for i in range(len(control_points_y)):
    #     plt.plot(t, by.basis_i(i, t))
    # plt.figure()
    plt.plot(control_points_x, control_points_y, 'o-')
    plt.plot(xs, ys)
    plt.plot(sci_xs, sci_ys, '-.')
    plt.grid(1)
    plt.show()