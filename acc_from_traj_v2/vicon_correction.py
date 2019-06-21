import numpy as np
def CorrectBiasedStamp(ts, threashold=0.7):
    dts = ts[1:] - ts[0:-1]
    dt_mean = dts.mean()
    dts_err = dts - dt_mean
    err_forward   = dts_err[:-1]
    err_backback  = dts_err[1:]
    comm = np.abs(err_forward + err_backback) / 2
    diff = np.abs(err_forward - err_backback) / 2
    cnt = 0
    for i in range(len(err_forward)):
        if diff[i] > 0.3 * dt_mean and comm[i] < 0.1 * dt_mean :
            idx_ts = i+1
            # print('biased stamp: {0}, cnt: {1}'.format(ts[idx_ts], cnt))
            # cnt += 1
            ts[idx_ts] = ts[idx_ts - 1] + dt_mean