
from dataclasses import dataclass
import numpy as np


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
    timepoint:        float
    sensorValues: list[int]

def getData():
    encoderData: list[EncoderData] = []
    sensorData: list[SensorData] = []

    with open("SystemidentificationScripts/response.txt", "r") as file:
        for line in file:
            timepoint, source, *readings = line[0:-1].split(",")
            if (source == "ROBOT"):
                wrist_angle, shoulder_angle, wrist_power, shoulder_power = [float(x) for x in readings][0:4]
                encData = EncoderData(float(timepoint), wrist_angle, shoulder_angle, wrist_power, shoulder_power)
                encoderData.append(encData)
            elif (source == "FORCE"):
                readings: list[str] = readings[0].split(" ")[3:]
                num = readings.count("")
                for _ in range(num): readings.remove("")

                sensData = SensorData(float(timepoint), [int(x) for x in readings[-8:]])
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

def getCLQ():

    C = np.matrix([
        -20.7056352,
        -10.94682257,
        -85.75310466,
        0.666177612,
        -1.124525726,
        0
    ], dtype=np.float64)

    L = np.matrix([
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0]
    ], dtype=np.float64)

    Q = np.matrix([])

    return C, L, Q

