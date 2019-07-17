import numpy as np

class Problem():
    def __init__():
        pass
    def d_cost_dx(self, x):
        return self.jac(x).transpose().dot(self.f(x)) + self.f(x).transpose().dot(self.jac(x))

    def cost(self, x):
        return self.f(x).transpose().dot(self.f(x))

def GaussNewton(problem, guess, step=20):
    x = guess
    for i in range(step):
        j = problem.jac(x)
        b = -j.transpose().dot(problem.residual(x))
        H = j.transpose().dot(j)
        print(H)
        update = np.linalg.solve(H, b)
        print('x: {}, ud: {}, dcost_dx:{}, cost: {}'.format(x, update, problem.d_cost_dx(x), problem.cost(x)))

        (rotvec, bias_sum) = Problem.parse_space_vector(update)
        x[0:3] = (R.from_rotvec(rotvec) * R.from_rotvec(x[0:3])).as_rotvec()
        x[3:6] += bias_sum
    return x