import fractions
import numpy as np
# import numpy.matrix as mat
np.set_printoptions(formatter={'all':lambda x: str(fractions.Fraction(x).limit_denominator())})

# g = np.matrix([[4],[2]])
# d = np.matrix([[-4],[-2]])
# A = np.matrix([[4,0],[0,2]])
g = np.matrix([[2],[-6]])
# d = np.matrix([[2],[4]])
A = np.matrix([[2,0],[0,6]])
# ans = -(g.transpose()*d)/(d.transpose() * A * d)
H = np.matrix([[2,1],[1,1]])
q = np.matrix([[10],[50]])/13
p = np.matrix([[5],[10]])/13
g2 = np.matrix([2,-1]).transpose()*18/13

H2 = H + p*p.transpose()/(p.transpose() * q) -  H*q*q.transpose()*H/(q.transpose()*H*q)
# print(H)
# print(q)
# print(ans)
# print(p*p.transpose()/(p.transpose() * q))
# print(-H2*g2)
d = -H2*g2

lam = -(g2.transpose()*d)/(d.transpose() * A * d)
print(lam)

for i in range(0,5):
    