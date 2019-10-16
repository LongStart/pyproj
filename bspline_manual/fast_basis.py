import matplotlib.pyplot as plt
import numpy as np

# def basis_0(knot_vector, i, u):
#         return np.array([float(knot_vector[i] <= v < knot_vector[i + 1]) for v in u])

# def basis_0_mat(knot_vector, u):
#     return np.vstack([[float(knot_vector[i] <= v < knot_vector[i + 1]) for v in u] for i in range(len(knot_vector) - 1)])

def basis_mat(knot_vector, u, degree):
    if 0 == degree:
        return np.vstack([[float(knot_vector[i] <= v < knot_vector[i + 1]) for v in u] for i in range(len(knot_vector) - 1)])
    b_last = basis_mat(knot_vector, u, degree - 1)
    coeff_0 = np.array([np.ones(len(u)) if np.isclose(knot_vector[i + degree], knot_vector[i]) \
        else (u - knot_vector[i])/(knot_vector[i + degree] - knot_vector[i]) \
        for i in range(len(knot_vector) - degree - 1)])
    coeff_1 = np.array([np.ones(len(u)) if np.isclose(knot_vector[i + degree + 1], knot_vector[i+1]) \
        else (knot_vector[i + degree + 1] - u)/(knot_vector[i + degree + 1] - knot_vector[i+1]) \
        for i in range(len(knot_vector) - degree - 1)])

    return b_last[:-1] * coeff_0 + b_last[1:] * coeff_1

if __name__ == "__main__":
    np.set_printoptions(precision=1, linewidth=np.inf)
    resolution = 200
    t = np.linspace(0, 50, resolution, endpoint=False)
    # knot_vector = np.array([4,4,4,4,5,6,7,7,7,7])
    # knot_vector = np.array([1,2,3,4,5,6,7,8,9])
    knot_vector = np.linspace(0, 30, 30)
    degree = 2

    b0_mat = basis_mat(knot_vector, t, 0)
    b1 = basis_mat(knot_vector, t, degree)
    # print(b1)
    if 1:
        plt.figure()
        plt.grid(1)
        for i in range(len(knot_vector) - degree - 1):
            plt.plot(t, b1[i])
        plt.show()
        