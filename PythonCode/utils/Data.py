
from dataclasses import dataclass

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

def logg_data(path: str, items: list[list]) -> None:
    if not path.endswith(".csv"):
        path += ".csv"

    with open(path, "a") as logg:
        for (a, v, t) in zip(*items):
            logg.write(f"{a},{v},{t}\n")

def getData():
    encoderData: list[EncoderData] = []
    sensorData: list[SensorData] = []

    with open("PythonCode/DATA/response.txt", "r") as file:
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

                sensData = SensorData(float(timepoint), [int(x) for x in readings[-12:-4]], float(readings[-4]), [float(x) for x in readings[-3:]])
                sensorData.append(sensData)
    return encoderData, sensorData