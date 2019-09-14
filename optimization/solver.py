import numpy as np

def GaussNewton(problem, guess, step=20, verbose=0):
    x = guess
    for i in range(step):
        j = problem.jac(x)
        b = -j.transpose().dot(problem.residual(x))
        H = j.transpose().dot(j)
        # print(j)
        update = np.linalg.solve(H, b)
        if verbose > 4:
            print('x: {}, ud: {}, dcost_dx:{}, res: {},j:\n{}, cost: {}'.format(x, update, problem.d_cost_dx(x), problem.residual(x), j, problem.cost(x)))
        elif verbose > 3:
            print('x: {}, ud: {}, dcost_dx:{}, j:\n{}, cost: {}'.format(x, update, problem.d_cost_dx(x), j, problem.cost(x)))
        elif verbose > 2:
            print('x: {}, j:\n{}, cost: {}'.format(x, j, problem.cost(x)))
        elif verbose > 1:
            print('x: {}, cost: {}'.format(x, problem.cost(x)))
        elif verbose > 0:
            print("it: {}".format(i))

        x = problem.update(x, update)
    return x


class Problem():
    def __init__():
        pass
    def d_cost_dx(self, x):
        # print("j.T dot res: ")
        # print(self.jac(x).T.dot(self.residual(x)))
        return self.jac(x).transpose().dot(self.residual(x)) + self.residual(x).transpose().dot(self.jac(x))

    def cost(self, x):
        return self.residual(x).transpose().dot(self.residual(x))

    def jac(self, x):
        raise ValueError("Problem.jac() is pure virtual!")

    def residual(self, x):
        raise ValueError("Problem.residual() is pure virtual!")

    def update(self, x, update):
        raise ValueError("Problem.update() is pure virtual!")

    def solve(self, guess, step=20, method='gn', verbose=0):
        if method == 'gn':
            return GaussNewton(self, guess, step, verbose=verbose)
        else:
            raise ValueError("Unknown method: {}".format(method))
