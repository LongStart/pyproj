from signal3d import Signal3d

class FrameSignal(Signal3d):
    def __init__(self, np_array, tf_buffer, frame_id):
        self.signal3d = Signal3d.from_np_array(np_array)
        self.tf_buffer = tf_buffer
        self.frame_id = frame_id
    
    @classmethod
    def from_np_array(cls, array, tf_buffer, frame_id='world'):
        return cls(array, tf_buffer, frame_id)

    @classmethod
    def from_vector(cls, vector, tf_buffer, t=None, frame_id='world'):
        if isinstance(t, int) or isinstance(t, float) or isinstance(t, np.ndarray):
            data = TimeConstantVector3d(t, vector)
            return cls(data, tf_buffer, frame_id)
        elif None == t:
            data = TimeConstantVector3d(0, vector)
            return cls(data, tf_buffer, frame_id)
    
    @classmethod
    def copy(cls, other):
        return cls(other.data(), other.tf_buffer, other.frame_id)

    # getter 
    def data(frame_id=None):
        pass

    # operator
    def __sub__(self, signal):
        pass

    def __add__(self, signal):
        pass

    def __mul__(self, value):
        pass

    def dot(self, signal):
        pass

if __name__ == '__main__':
    print("ttt")