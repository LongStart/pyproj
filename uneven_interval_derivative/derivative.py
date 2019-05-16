import numpy as np


def derivative(p, t):
    d2t = t[2:] - t[:-2]
    dp = p[1:] - p[:-1]
    dt = t[1:] - t[:-1]
    v = 1 / d2t * (dp[1:]*dt[:-1]/dt[1:] + dp[:-1]*dt[1:]/dt[:-1])
    v.insert(0, dp[0]/dt[0])
    return v

if __name__ == '__main__':
    p = np.array([1,2,3,2,1,2,3,2])
    t = np.array([1,2,3,4,5,6,7,8])
    print(derivative(p,t))