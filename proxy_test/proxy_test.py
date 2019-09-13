
class Value():
    def __init__(self, value):
        self._value = value
    
    def a(self):
        return self._value
    def b(self, value):
        self._value = value
    value = property(a, b)

class Arr():
    def __init__(self):
        self._value = [1,2,3,4]
    
    def a(self, i):
        return self._value[i]


if __name__ == "__main__":
    c = Value(2)
    c.value = 3
    print(c.value)

    t = Arr()
    t.a(0) = 2
    
    print(t._value)