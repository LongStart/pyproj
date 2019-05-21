import numpy as np
from dsp import *
import operator

class Signal3d():
    
    def __init__(self, param):
        if(isinstance(param, np.ndarray)):
            self.data = param
        elif(isinstance(param, Signal3d)):
            self.data = param.data
        else:
            raise TypeError('fuck?')

    # getter
    def t(self):
        return np.array(self.data[0])
    def xyz(self):
        return np.array(self.data[1:])
    
    # operator
    def __sub__(self, signal):
        return Signal3d(UnalignedOperate3d(self.data, signal.data, operator.sub))

    def __add__(self, signal):
        return Signal3d(UnalignedOperate3d(self.data, signal.data, operator.add))

    def __mul__(self, value):
        if(isinstance(value, type(1.)) or isinstance(value, type(1))):
            xyz = self.data[1:] * value
            return Signal3d(np.array([self.t(), xyz[0], xyz[1], xyz[2]]))
        else:
            raise TypeError('Cannot handle type: {0}'.format(type(value)))

    def dot(self, signal):
        return Signal3d(UnalignedOperate3d(self.data, signal.data, operator.mul))

    def Rotate(self, signal):
        if isinstance(signal, Signal3d):
            return Signal3d(UnalignedRotate(self.data, signal.data))
        if isinstance(signal, np.ndarray) and 3 == len(signal):
            return Signal3d(Rotate(self.data, signal))
        else:
            raise TypeError("not handle: {0}".format(type(signal)))
    
    #dsp
    def Derivative(self):
        return Signal3d(Derivative3d(self.data))

    def Integral(self):
        return Signal3d(Integral3d(self.data))

    def MovingAverage(self, kernel_size=3):
        return Signal3d(MovingAverage3d(self.data, kernel_size))
    
    def Midfilter(self, kernel_size=5):
        return Signal3d(Medfilter3d(self.data, kernel_size))

    def LFilter(self, critical_freq, order=3):
        return Signal3d(LFilter3d(self.data, order, critical_freq))

    