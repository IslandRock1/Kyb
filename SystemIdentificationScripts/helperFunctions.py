
from dataclasses import dataclass
import numpy as np
import pandas as pd


def logg_data(path: str, items: list[list]) -> None:
    """

    :param path: Name of a file, with or without an extension
    :param items: list of angle/angle velocity, voltage, time
    :return:
    """
    if not path.endswith(".csv"):
        path += ".csv"

    with open(path, "a") as logg:
        for (a, v, t) in zip(*items):
            logg.write(f"{a},{v},{t}\n")

@dataclass
class EncoderData:
    timepoint:      float
    wrist_angle:    float
    shoulder_angle: float
    wrist_power:    float
    shoulder_power: float

@dataclass
class SensorData:
    timepoint    :       float
    sensorValues :   list[int]
    weight       :       float
    centerOfMass : list[float]

def getData():
    encoderData: list[EncoderData] = []
    sensorData: list[SensorData] = []

    with open("SystemidentificationScripts/response.txt", "r") as file:
        for line in file:
            timepoint, source, *readings = line[0:-1].split(",")
            if (source == "ROBOT"):
                try:
                    wrist_angle, shoulder_angle, wrist_power, shoulder_power = [float(x) for x in readings][0:4]
                    encData = EncoderData(float(timepoint), wrist_angle, shoulder_angle, wrist_power, shoulder_power)
                    encoderData.append(encData)
                except: pass
            elif (source == "FORCE"):
                readings: list[str] = readings[0].split(" ")[3:]
                num = readings.count("")
                for _ in range(num): readings.remove("")

                # print(readings)
                sensData = SensorData(float(timepoint), [int(x) for x in readings[-12:-4]], float(readings[-4]), [float(x) for x in readings[-3:]])
                # print(sensData)
                sensorData.append(sensData)
    return encoderData, sensorData

def Rx(theta):
    return np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])

def Ry(theta):
    return np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)],
    ])

def SensorToWorld(theta_shoulder, theta_wrist):
    return Ry(theta_wrist) @ Rx(theta_shoulder)

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

def formatForceLabel(x):
    out = round(float(x), 2)
    return f" {out: .2f}"

def getCLQ():

    # This code is taken from Jon's Python scrip "4_validation_quadratic.py"
    n_sensors = 8
    df = pd.read_csv(f'params_linearregression_quadratic.csv')
    C = df[['C']].values # Shape: (6, 1)
    L = df[['L_s0','L_s1','L_s2','L_s3','L_s4','L_s5','L_s6','L_s7']].values # Shape: (6, 8)
    Q_cols = [f'Q_s{i}s{j}' for i in range(n_sensors) for j in range(i, n_sensors)]
    Q = df[Q_cols].values  # Shape: (6, 36)

    return C, L, Q

