
class DynamicConstruct(object):
    def __init__(self, val):
        self.val = val
        super(DynamicConstruct, self).__setattr__('_val', val)
    
    def __getattr__(self, name):
        if name == 'dim':
            return self._dim

if __name__ == "__main__":
    print('start')
    # dc = DynamicConstruct()
    a = DeprecationWarning.dim