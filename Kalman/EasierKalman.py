# based on examples here: https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html

import numpy as np
from filterpy.kalman import KalmanFilter

def make_contact_wrench_kf(Q=None, R=None):
    kf = KalmanFilter(dim_x=6, dim_z=6)

    kf.F = np.eye(6)
    kf.H = np.eye(6)

    kf.Q = Q if Q is not None else np.eye(6) * 1e-4
    kf.R = R if R is not None else np.eye(6) * 1e-2

    kf.x = np.zeros(6,)
    kf.P *= 1.0

    return kf


kf = make_contact_wrench_kf()

while data_available:   # må være kobla til en målinger
    zf = get_fts_measurement() # tar inn en 1D array med lengde 6, float
    kf.predict()
    kf.update(zf)

    gravity_offset = np.array([...])    # noe data her

    wrench_est = (kf.x.copy().ravel() - gravity_offset)    # [Fx, Fy, Fz, Tx, Ty, Tz]