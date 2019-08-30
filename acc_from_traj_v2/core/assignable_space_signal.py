import numpy as np

class SequenceXd(object):
    def __init__(self, dim, data):
        self._data = np.array([data]) if data.ndim == 1 else data

    def set(self, data):
        if data.ndim == 1:
            assert np.shape(self._data) == (1,len(data))
            self._data = np.array([data])
            return
        assert np.shape(self._data) == np.shape(data)
        self._data = data

    def get(self):
        if np.shape(self._data)[0] == 1:
            return np.array(self._data[0])
        return np.array(self._data)

class SignalXd(object):
    def __init__(self, t_vals, val_name='vals', dim=None):
        if dim is None:
            dim = np.shape(t_vals)[0] - 1
            assert dim > 0
        else:
            assert np.shape(t_vals) == (dim + 1, len(t_vals[0]))
        super(SignalXd, self).__setattr__('_t', SequenceXd(1, t_vals[0]))
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

    @staticmethod
    def from_t_vals(cls, t, vals=None):
        if vals is None:
            vals = np.zeros((cls.__dim__, len(t)))
        return cls(np.vstack((t, vals)))

    def init_signal(obj, t_vals):
        super(obj.__class__, obj).__init__(t_vals, obj.__class__.__val_name__, obj.__class__.__dim__)

def RegisterSignalClass(cls):
    setattr(cls, 'from_t_' + cls.__val_name__, classmethod(SignalXd.from_t_vals))

class Signal1d(SignalXd):
    __val_name__ = 'x'
    __dim__ = 1
    def __init__(self, t_vals):
        super(self.__class__, self).__init__(t_vals, self.__class__.__val_name__, self.__class__.__dim__)
RegisterSignalClass(Signal1d)

class Signal3d(SignalXd):
    __val_name__ = 'xyz'
    __dim__ = 3
    def __init__(self, t_vals):
        super(self.__class__, self).__init__(t_vals, self.__class__.__val_name__, self.__class__.__dim__)
RegisterSignalClass(Signal3d)

class Signal4d(SignalXd):
    __val_name__ = 'xyzw'
    __dim__ = 4
    def __init__(self, t_vals):
        super(self.__class__, self).__init__(t_vals, self.__class__.__val_name__, self.__class__.__dim__)
RegisterSignalClass(Signal4d)

class Trajectory3d(object):
    def __init__(self, t_vals):
        assert np.shape(t_vals) == (8, len(t_vals[0]))
        super(Trajectory3d, self).__setattr__('_t', SequenceXd(1, t_vals[0]))
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
