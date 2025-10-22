
import numpy as np
import matplotlib.pyplot as plt

#A = np.matrix([
#    [1, -4.162 * 1e-5, 2.443 * 1e-5, 3.276 * 1e-5],
#    [0.001888, 0.9988, 0.2854, -0.06767],
#    [0.0002122, -0.001463, -0.9093, 0.6212],
#    [-0.0001744, 0.003023, -0.05208, -0.7061]
#])

A = np.matrix([
    [-0.0002669, -0.01271, -0.05484, 0.1313],
    [1.437, 1.569, -1453, 3327],
    [0.1271, 36.61, -3879, 1.396e+04],
    [-1.642, -31.89, 3456, -1.96e+04]
])


B = np.matrix([
    [0.0002345],
    [5.72],
    [22.5],
    [-29.94]
])

C = np.matrix([1.613 * 1e-6, 23.03, 94.47, -206.4])

K = np.matrix([
    [0.000662],
    [-0.5242],
    [5.122],
    [-2.479]
])

x = np.matrix([
    [0.0],
    [0.0],
    [0.0],
    [0.0]
])

t0 = None
t = [0, 0]
measured = [0, 0]
calculated = [0, 0]
motor_power = [0, 0]

# Measured, motor_power, time
with open("data_raw2.csv", "r") as file:
    for line in file:
        measure, power, time = line.split(',')

        if (t0 is None):
            t0 = float(time)

        t.append((float(time) - t0) / 1000000.0)
        motor_power.append(float(power) * 12)
        measured.append(float(measure))

        dx = A @ x + B * (float(power) * 12)
        x += dx * 0.000971

        calcula = C @ x
        calculated.append(calcula[0,0])

plt.plot(t, measured, label = "Measured")
plt.plot(t, calculated, label = "Calculated")
plt.legend()
plt.show()