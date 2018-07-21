import matplotlib.pyplot as plt

a = 1
k = 50
n = 20
i = [x for x in range(1,n+1)]
X = [a * (v+k)**0.5 for v in i]
y = [0]*n

plt.plot(X,y,'x')
plt.show()

