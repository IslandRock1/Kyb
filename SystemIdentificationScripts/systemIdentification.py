
import numpy as np
from sippy_unipi import system_identification as sysid
from sippy_unipi.armaxMIMO import ARMAX_MIMO_model as model 

# Suppose u = voltage input, y = angular speed output
# Both are numpy arrays of equal length
# Ts = sampling time

voltage = []
angular_speed = []
time = []

with open("data_logg_speedy.csv", "r") as logg:

    for line in logg:
        a, v, t = line.split(",")
        angular_speed.append(float(a))
        voltage.append(12.0 * float(v))
        time.append(float(t) / 1000000.0)
        # print(float(a), 12.0 * float(v), float(t) / 1000000.0)

voltage = np.array(voltage)
angular_speed = np.array(angular_speed)
time = np.array(time)

Ts = np.mean(np.diff(time))

G: model = sysid(
    y=angular_speed,
    u=voltage,
    id_method='ARMAX',
    tsample=Ts,
    SS_fixed_order=2
)

print("Num: ", G.NUMERATOR[0][0][0])
print("Den: ", G.DENOMINATOR[0][0][0])

