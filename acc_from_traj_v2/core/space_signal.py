import numpy as np

class Sequence():
    def __init__()

class SequenceXd():
    def __init__(self, dim, data):
        assert np.shape(data) == (dim, len(data[0]))
        self._data = data
        self._len = len(data[0])

    def set(self, data):
        assert np.shape(self._data) == np.shape(data)
        self._data = data
    
    def get(self):
        return self._data

class TimeSequence():
    def __init__(self, data):
        assert np.shape(data) == (len(data),)
        self._data = data

    def set(self, data):
        assert np.shape(self._data) == np.shape(data)
        self._data = data
    
    def get(self):
        return self._data

class Signal3d():
    def __init__(self, t_vals):
        self.dim = 3
        assert np.shape(t_vals) == (self.dim + 1, len(t_vals[0]))
        self.t = TimeSequence(1, t_vals[0])
        self.vals = SequenceXd(self.dim, t_vals[1:])

    def __getattribute__(self, name):
        if name == 't':
            return self.t

    def __call__(self):
        return np.vstack((self.t(), self.xyz()))

    @classmethod
    def from_t_xyz(cls, t, xyz=None):
        if xyz is None:
            xyz = np.zeros((self.dim, len(t)))
        return cls(np.vstack((t, xyz)))

    @property
    def t_xyz(self):
        return self


class Signal4d():
    def __init__(self, t_xyzw):
        self.dim = 4
        assert np.shape(t_xyzw) == (self.dim + 1, len(t_xyzw[0]))
        self.t = TimeSequence(1, t_xyzw[0])
        self.xyzw = SequenceXd(self.dim, t_xyzw[1:])
    
    def __call__(self):
        return np.vstack((self.t(), self.xyzw()))

    @classmethod
    def from_t_xyzw(cls, t, xyzw=None):
        if xyzw is None:
            xyzw = np.zeros((self.dim, len(t)))
        return cls(np.vstack((t, xyzw)))

    @property
    def t_xyzw(self):
        return self
        
class Trajectory3d():
    def __init__(self, t_xyz_xyzw):
        assert np.shape(t_xyz_xyzw) == (8, len(t_xyz_xyzw[0]))
        self.t = TimeSequence(1, t_xyz_xyzw[0])
        self.xyz = SequenceXd(3, t_xyz_xyzw[1:4])
        self.xyzw = SequenceXd(4, t_xyz_xyzw[4:])

    def __call__(self):
        return np.vstack((self.t(), self.xyz(), self.xyzw()))

    @classmethod
    def from_t_xyz_xyzw(cls, t, xyz=None, xyzw=None):
        if xyz is None:
            xyz = np.zeros((3, len(t)))
        if xyzw is None:
            xyzw = np.zeros((4, len(t)))
        return cls(np.vstack((t, xyz, xyzw)))

    @property 
    def t_xyz(self):
        return Signal3d.from_t_xyz(self.t(), self.xyz())

    @property 
    def t_xyzw(self):
        return Signal4d.from_t_xyzw(self.t(), self.xyzw())

    @property
    def t_xyz_xyzw(self):
        return self
