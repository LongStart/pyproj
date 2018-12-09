from math import *
import matplotlib.pyplot as plt

def combination(x,y):
    return factorial(x)/factorial(y)/factorial(x-y)

def function(x):
    sum = 0
    for i in range(2,7):
        sum += combination(20, i) * pow(x,i) * pow(1-x, 20 - i)
    # print(sum)
    return 1 - sum

for i in range(1,10):
    x = i * 0.1
    print("f({}) = {}".format(x, function(x)))

step_len = 1e-2
xs = [step_len * v for v in range(0,int(1 / step_len))]
ys = [function(v) for v in xs]
plt.plot(xs, ys)

plt.ylabel('β(p)')
plt.xlabel('p')
plt.grid(1)
plt.show()

