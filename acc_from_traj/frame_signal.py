from signal3d import Signal3d

class FrameSignal(Signal3d):
    def __init__(self, np_array, tf_buffer, frame_id='world'):
        self.data = np_array
        self.tf_buffer = tf_buffer
        self.frame_id = frame_id
    
    def 

if __name__ == '__main__':
    print("ttt")