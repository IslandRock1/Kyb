import serial
from time import perf_counter
import matplotlib.pyplot as plt

# Open serial port (adjust COM port and baud rate)
ser = serial.Serial('COM3', 1_152_000, timeout=1)

angle = []
voltage = []
time_vals = []

t0 = perf_counter()
while perf_counter() - t0 < 20:
    try:
        # Read one line (until '\n')
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue  # skip empty lines

        # Split by comma
        parts = line.split(',')
        if len(parts) != 3:
            continue  # skip malformed lines

        # Convert values safely
        try:
            angle_v = int(parts[0])
            voltage_v = float(parts[1])
            time_v = int(parts[2])
        except ValueError:
            continue  # skip lines with conversion errors

        # Store values
        angle.append(angle_v)
        voltage.append(voltage_v)
        time_vals.append(time_v)

    except KeyboardInterrupt:
        break  # allow manual exit
t1 = perf_counter()
print(f"Total listening time: {(t1 - t0):.2f} | Avg time per reading: {1000000 * (t1 - t0) / len(angle):.0f} us.")

print(f"Num measurments: {len(angle)} | {len(voltage)} | {len(time_vals)}")

# Compute average delta for first 100 intervals
if len(time_vals) > 1:
    delta = [time_vals[x] - time_vals[x - 1] for x in range(1, len(time_vals))]
    print("Average interval:", sum(delta) / len(delta))

# Log raw data to file (same format as before)
with open("data_logg.csv", "a") as logg:
    for (a, v, t) in zip(angle, voltage, time_vals):
        logg.write(f"{a},{v},{t}\n")

# Plot as before
plt.plot(time_vals, angle, label = "Angle")
plt.legend()
plt.show()

#Processing part from the second script

angle_proc = angle[:]
voltage_proc = voltage[:]
time = time_vals[:]            # reuse original variable name from script 2

def senter_diff(v0, v1, t):
    return (v1 - v0) / t

pi = 3.1415926

if len(time) >= 3:
    speed = [(2 * pi * 1000000 / 4096) * senter_diff(angle_proc[i - 1], angle_proc[i + 1], time[i + 1] - time[i - 1]) for i in range(1, len(time) - 1)]
    print(len(speed))

    # Align voltage and time with centered-difference result (drop ends)
    voltage_proc.pop()
    voltage_proc.pop(0)

    time.pop()
    time.pop(0)

    with open("data_logg_speedy.csv", "a") as logg:
        for (a, v, t) in zip(speed, voltage_proc, time):
            logg.write(f"{a},{v},{t}\n")
else:
    print("Not enough samples to compute centered-difference speed.")


ser.close()
