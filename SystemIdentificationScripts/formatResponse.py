import numpy as np

from helperFunctions import SensorData, EncoderData, getData, getForceVector, getMassVector, SensorToWorldFromSlides

def syncData(encoderData: list[EncoderData], sensorData: list[SensorData]):
    outSensor = []
    outEncoder = []

    i = 0
    for currSensorData in sensorData:
        if (i % 1000 == 0):
            print(f"Num Sensordata: {len(sensorData)} | Num Encoderdata: {len(encoderData) - i} | i: {i}")

        while (len(encoderData) > (i + 10)) and (encoderData[i].timepoint < currSensorData.timepoint):
            i += 1

        if (len(encoderData) < (i + 10)):
            break

        outSensor.append(currSensorData)
        outEncoder.append(encoderData[i])

    return (outEncoder, outSensor)


def main():
    data = getData()
    encoderDataL, sensorDataL = syncData(data[0], data[1])

    lsdt = []
    with open("syncronized.csv", "w") as f:
        f.write("Timestamp,Fx,Fy,Fz,Mx,My,Mz,s0,s1,s2,s3,s4,s5,s6,s7\n")
        for (encoderData, sensorData) in zip(encoderDataL, sensorDataL):
            dt = encoderData.timepoint - sensorData.timepoint
            lsdt.append(dt)

            Rsw = SensorToWorldFromSlides(encoderData.shoulder_angle, encoderData.wrist_angle)

            forceVector = getForceVector(sensorData.weight, Rsw).flatten()
            massVector = getMassVector(np.matrix([[sensorData.centerOfMass[0]], [sensorData.centerOfMass[1]], [sensorData.centerOfMass[2]]]), forceVector).flatten()
            f.write(f"{encoderData.timepoint},{forceVector[0,0]},{forceVector[0,1]},{forceVector[0,2]},{massVector[0]},{massVector[1]},{massVector[2]},{sensorData.sensorValues[0]},{sensorData.sensorValues[1]},{sensorData.sensorValues[2]},{sensorData.sensorValues[3]},{sensorData.sensorValues[4]},{sensorData.sensorValues[7]},{sensorData.sensorValues[6]},{sensorData.sensorValues[5]}\n")

if __name__ == "__main__":
    print()
    main()
    # test()