
from dataclasses import dataclass
import matplotlib.pyplot as plt

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

def plotSensorData(sensorData: list[SensorData]):
    x_values = [x.timepoint - sensorData[0].timepoint for x in sensorData]
    for i in range(8):
        y_values = [x.sensorValues[i] for x in sensorData]
        plt.plot(x_values, y_values, label = f"Sensor {i}")

    plt.title("Sensor Values")
    plt.xlabel("Time (s)")
    plt.ylabel("Sensor value (0-1024)")

    plt.grid()
    plt.legend()
    plt.show()

def plotEncoderData(encoderData: list[EncoderData]):
    x_values = [x.timepoint - encoderData[0].timepoint for x in encoderData]
    y_values_wrist = [x.wrist_angle for x in encoderData]
    y_values_shoulder = [x.shoulder_angle for x in encoderData]

    y_values_wrist_power = [x.wrist_power * 12.0 / 255.0 for x in encoderData]
    y_values_shoulder_power = [x.shoulder_power * 12.0 / 255.0 for x in encoderData]

    # plt.plot(x_values, y_values_wrist, label = "Wrist angle")
    # plt.plot(x_values, y_values_shoulder, label = "Shoulder angle")

    # plt.plot(x_values, y_values_wrist_power, label = "Wrist power")
    # plt.plot(x_values, y_values_shoulder_power, label = "Shoulder power")

    fig, ax1 = plt.subplots()
    color = 'tab:red'
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Shoulder angle (deg)', color=color)
    ax1.plot(x_values, y_values_shoulder, color=color)
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second Axes that shares the same x-axis

    color = 'tab:blue'
    ax2.set_ylabel('Shoulder power (V)', color=color)  # we already handled the x-label with ax1
    ax2.plot(x_values, y_values_shoulder_power, color=color)
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    plt.title("Angle vs Power")
    plt.grid()
    plt.show()

def main():
    print()
    encoderData, sensorData = getData()

    #plotSensorData(sensorData)
    plotEncoderData(encoderData)

if __name__ == "__main__":
    main()