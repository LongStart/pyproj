import numpy as np

class Sequence(object):
    def __init__(self):
        self._data = None

    def set(self, data):
        assert np.shape(self._data) == np.shape(data)
        self._data = data
    
    def get(self):
        return np.array(self._data)

class SequenceXd(Sequence):
    def __init__(self, dim, data):
        super(SequenceXd, self).__init__()
        assert np.shape(data) == (dim, len(data[0]))
        self._data = data
        self._len = len(data[0])

class TimeSequence(Sequence):
    def __init__(self, data):
        assert np.shape(data) == (len(data),)
        self._data = data

class SignalXd(object):
    def __init__(self, t_vals, val_name='vals', dim=None):
        if dim is None:
            dim = np.shape(t_vals)[0] - 1
            assert dim > 0
        else:
            assert np.shape(t_vals) == (dim + 1, len(t_vals[0]))
        super(SignalXd, self).__setattr__('_t', TimeSequence(t_vals[0]))
        super(SignalXd, self).__setattr__('_' + val_name, SequenceXd(dim, t_vals[1:]))
        super(SignalXd, self).__setattr__('_dim', dim)
        super(SignalXd, self).__setattr__('_val_name', val_name)

    def __getattr__(self, name):
        if name == 'dim':
            return self._dim
        if name == 't_' + self._val_name:
            return np.vstack((self._t.get(), self.__getattribute__('_' + self._val_name).get()))
        else:
            return self.__getattribute__('_' + name).get()

    def __setattr__(self, name, value):
        self.__getattribute__('_' + name).set(value)

    @classmethod
    def combine_t_vals(cls, t, vals=None):
        if vals is None:
            vals = np.zeros((cls.dim(), len(t)))
        return np.vstack((t, vals))

    @classmethod
    def from_t_vals(cls, t, vals=None, val_name='vals', dim=None):
        assert(dim is not None or vals is not None)
        return cls(SignalXd.combine_t_vals(t, vals), val_name, dim)

class Signal3d(SignalXd):
    def __init__(self, t_vals):
        super(Signal3d, self).__init__(t_vals, 'xyz', 3)

    @classmethod
    def from_t_xyz(cls, t, vals=None):
        return cls(SignalXd.combine_t_vals(t, vals))

class Signal4d(SignalXd):
    def __init__(self, t_vals):
        super(Signal3d, self).__init__(t_vals, 'xyzw', 4)

    @classmethod
    def from_t_xyzw(cls, t, vals=None):
        return cls(SignalXd.combine_t_vals(t, vals))
        
class Trajectory3d(object):
    def __init__(self, t_vals):
        assert np.shape(t_vals) == (8, len(t_vals[0]))
        super(Trajectory3d, self).__setattr__('_t', TimeSequence(t_vals[0]))
        super(Trajectory3d, self).__setattr__('_xyz', SequenceXd(3, t_vals[1:4]))
        super(Trajectory3d, self).__setattr__('_xyzw', SequenceXd(4, t_vals[4:]))

    def __getattr__(self, name):
        if name == 't_xyz':
            return np.vstack((self._t.get(), self._xyz.get()))
        if name == 't_xyzw':
            return np.vstack((self._t.get(), self._xyzw.get()))
        if name == 't_xyz_xyzw':
            return np.vstack((self._t.get(), self._xyz.get(), self._xyzw.get()))
        return self.__getattribute__('_' + name).get()

    def __setattr__(self, name, value):
        if name == 't_xyz':
            self._t.set(value[0])
            self._xyz.set(value[1:])
            return 
        if name == 't_xyzw':
            self._t.set(value[0])
            self._xyzw.set(value[1:])
            return 
        if name == 't_xyz_xyzw':
            self._t.set(value[0])
            self._xyz.set(value[1:4])
            self._xyzw.set(value[4:])
            return 
        self.__getattribute__('_' + name).set(value)

    @classmethod
    def from_t_xyz_xyzw(cls, t, xyz=None, xyzw=None):
        if xyz is None:
            xyz = np.zeros((3, len(t)))
        if xyzw is None:
            xyzw = np.zeros((4, len(t)))
        return cls(np.vstack((t, xyz, xyzw)))
