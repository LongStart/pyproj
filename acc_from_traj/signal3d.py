import numpy as np
from dsp import *
import operator

class Signal3d():
    """ 
        All argument of the interfaces should be Signal3d 
    """
    def __init__(self, np_array):
            self.data = np_array
    
    @classmethod
    def from_np_array(cls, array):
        return cls(np_array = array)

    @classmethod
    def from_vector(cls, vector, t=None):
        if isinstance(t, int) or isinstance(t, float) or isinstance(t, np.ndarray):
            data = TimeConstantVector3d(t, vector)
            return cls(np_array = data)
        elif None == t:
            data = TimeConstantVector3d(0, vector)
            return cls(np_array = data)
    
    @classmethod
    def copy(cls, other):
        return cls(np_array = other.data)




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

    # def Rotate(self, signal):
    #     return Signal3d(UnalignedRotate(self.data, signal.data))
    def _Rotate(self, txyzw):
        assert(len(self.data.transpose()) == len(txyzw.transpose()))
        result = np.array(self.data)
        R.from_quat(txyzw.transpose()[1:]).apply(result.transpose()[1:])
        return result
    
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

    