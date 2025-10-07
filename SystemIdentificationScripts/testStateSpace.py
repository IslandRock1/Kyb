
import numpy as np
import matplotlib.pyplot as plt

A = np.matrix([
    [-11.91, -13.8, -14.98, -9.787],
    [-11.22, -14.6, -22.18, -13.88],
    [-2.182, -1.319, -10.83, -14.48],
    [-0.4334, 1.176, 3.084, -1.651]
])

B = np.matrix([
    [-1.602],
    [-0.8922],
    [0.7668],
    [-0.148]
])

C = np.matrix([-1.764, -0.3403, -0.3163, -0.1236])
x = np.matrix([
    [0.0],
    [0.0],
    [0.0],
    [0.0]
])

t0 = None
t = []
measured = []
calculated = []
motor_power = []

# Measured, motor_power, time
with open("data_processed.csv", "r") as file:
    for line in file:
        measure, power, time = line.split(',')


        if (t0 is None):
            t0 = float(time)

        t.append((float(time) - t0) / 1000000.0)
        motor_power.append(float(power))
        measured.append(float(measure))

        dx = A @ x + B * float(power)
        x += dx * 0.1

        calcula = C @ x
        calculated.append(calcula[0,0])

plt.plot(t, measured, label = "Measured")
plt.plot(t, calculated, label = "Calculated")
plt.show()