import numpy as np

class Sequence3d():
    def __init__(self, xyz):
        self.xyz = xyz

class Sequence4d():
    def __init__(self, wxyz):
        self.wxyz = wxyz

class Signal3d():
    def __init__(self, t, xyz):
        self.xyz = xyz
        self.t = t

class SignalRot():
    def __init__(self, t, wxyz):
        self.wxyz = wxyz
        self.t = 0

class Trajectory3d():
    def __init__(self, t, pos, rot):
        self.rot = rot
        self.pos = pos
        self.t = t