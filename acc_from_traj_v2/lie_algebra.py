import numpy as np

def __hat__(vs):
    """
    :param v: 3x1 vector
    :return: 3x3 skew symmetric matrix
    """
    return np.array([[[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]] for v in vs])
    

def hat(v):
    is_single = False
    rotvec = np.asarray(v, dtype=float)

    if rotvec.ndim not in [1, 2] or rotvec.shape[-1] != 3:
        raise ValueError("Expected `rot_vec` to have shape (3,) "
    "or (N, 3), got {}".format(rotvec.shape))

    if rotvec.shape == (3,):
        rotvec = rotvec[None, :]
        is_single = True
    
    if is_single:
        return __hat__(np.array([v]))[0]
    return __hat__(v)

def vee(m):
    """
    :param m: 3x3 skew symmetric matrix
    :return: 3x1 vector
    """
    return np.array([-m[1, 2], m[0, 2], -m[0, 1]])