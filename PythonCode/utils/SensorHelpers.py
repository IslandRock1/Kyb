
import numpy as np
import pandas as pd

def SensorToWorldFromSlides(theta_shoulder, theta_wrist):
    t1 = theta_shoulder
    t2 = theta_wrist

    sin = np.sin
    cos = np.cos

    return np.matrix([
        [-sin(t2), -cos(t2), 0],
        [sin(t1) * cos(t2), -sin(t1) * sin(t2), cos(t1)],
        [-cos(t1) * cos(t2), cos(t1) * sin(t2), sin(t1)],
    ])

def getForceVector(mass, Rsw):
    return np.linalg.inv(Rsw) @ np.matrix([[0], [0], [-mass * 9.81]])

def getMassVector(rs, forceVector):
    return np.linalg.cross(rs.flatten(), forceVector.flatten())

def getCLQ():

    # This code is taken from Jon's Python scrip "4_validation_quadratic.py"
    n_sensors = 8
    df = pd.read_csv(f'PythonCode/DATA/params_linearregression_quadratic.csv')
    C = df[['C']].values # Shape: (6, 1)
    L = df[['L_s0','L_s1','L_s2','L_s3','L_s4','L_s5','L_s6','L_s7']].values # Shape: (6, 8)
    Q_cols = [f'Q_s{i}s{j}' for i in range(n_sensors) for j in range(i, n_sensors)]
    Q = df[Q_cols].values  # Shape: (6, 36)

    return C, L, Q