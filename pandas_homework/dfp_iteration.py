import fractions
import numpy as np
np.set_printoptions(formatter={'all':lambda x: str(fractions.Fraction(x).limit_denominator())})

# x = np.matrix([2,1]).transpose()
# H = np.matrix([[1,0],[0,1]])
# A = np.matrix([[4,0],[0,2]])
# g = np.matrix([4*(float(x[0]) - 1), 2*float(x[1])]).transpose()

x = np.matrix([1,-1]).transpose()
H = np.matrix([[2,1],[1,1]])
A = np.matrix([[2,0],[0,6]])
def calc_g(x):
    return np.matrix([2*np.asscalar(x[0]), 6*np.asscalar(x[1])]).transpose()

g = calc_g(x)
g_prev = g

d = -H * g

lam = np.asscalar(-(g.transpose()*d)/(d.transpose() * A * d))
x = x + lam * d

print('x:')
print(x)

for i in range(1,3):
    print('===========i: {}==========='.format(i))
    print('g:')
    print(g)
    g = calc_g(x)
    p = lam * d
    q = -g_prev + g
    print("p,q: ")
    print(p)
    print(q)
    H = H + p*p.transpose()/(p.transpose() * q) -  H*q*q.transpose()*H/(q.transpose()*H*q)
    print('H:')
    print(H)
    d = -H * g
    print('d:')
    print(d)
    
    lam_mat = -(g.transpose()*d)/(d.transpose() * A * d)
    lam = np.asscalar(lam_mat)
    
    print('lam:')
    print(lam_mat)
    print("lam * d: " )
    print(lam * d)
    x = x + lam * d
    g_prev = g
    
    print('x:')
    print(x)
