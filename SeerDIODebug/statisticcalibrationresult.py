import numpy
from matplotlib import pyplot

ff = open('.\\caliresult\\result0.txt')
resultlist = []
for line in ff:
    resultlist += [int(line)]

# print(resultlist)
x = numpy.array(resultlist)
ax0 = pyplot.hist(x, 40, normed=1, histtype='bar',
                  facecolor='yellowgreen', alpha=1)
pyplot.show()
