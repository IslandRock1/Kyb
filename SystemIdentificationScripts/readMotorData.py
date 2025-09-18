import serial
from time import perf_counter
import matplotlib.pyplot as plt

import helperFunctions

# Open serial port (adjust COM port and baud rate)
ser = serial.Serial('COM3', 1_152_000, timeout=1)

angle = []
voltage = []
time_vals = []

ser.write(b'Ready\n')

timeOfMaxVoltage = None
t0 = perf_counter()
try:
    while True:
        if (timeOfMaxVoltage is not None) and (perf_counter() - timeOfMaxVoltage > 3.0):
            break

        if ser.in_waiting:
            # Read one line (until '\n')
            line = ser.readline().decode(errors='ignore').strip()
            print(line)
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

                if (voltage_v >0.99) and (timeOfMaxVoltage is None):
                    timeOfMaxVoltage = perf_counter()
            except ValueError:
                continue  # skip lines with conversion errors

            # Store values
            angle.append(angle_v)
            voltage.append(voltage_v)
            time_vals.append(time_v)

except KeyboardInterrupt:
    print("Manually stopped\n")
finally:
    t1 = perf_counter()
    ser.close()
    print(f"Total listening time: {(t1 - t0):.2f} | Avg time per reading: {1000000 * (t1 - t0) / len(angle):.0f} us.")
    print(f"Num measurements: {len(angle)} | {len(voltage)} | {len(time_vals)}")

    if len(time_vals) > 1:
        delta = [time_vals[x] - time_vals[x - 1] for x in range(1, len(time_vals))]
        print("Average interval:", sum(delta) / len(delta))

    # Log raw data to file
    helperFunctions.logg_data("data_raw", [angle, voltage, time_vals])

    plt.plot(time_vals, angle, label = "Angle")
    plt.legend()
    plt.show()

    def center_diff(v0, v1, t0, t1):
        return (v1 - v0) / (t1 - t0)

    pi = 3.1415926
    if len(time_vals) >= 3:
        conversionFactor = 2 * pi * 1e6 / 4096
        speed = [
            conversionFactor * center_diff(
            angle[i - 1], angle[i + 1], time_vals[i - 1], time_vals[i + 1]
            )
            for i in range(1, len(time_vals) - 1)
        ]
        print(len(speed))

        # Align voltage and time with centered-difference result (drop ends)
        voltage = voltage[1:-1]
        time_vals = time_vals[1:-1]

        helperFunctions.logg_data("data_processed", [speed, voltage, time_vals])
    else:
        print("Not enough samples to compute centered-difference speed.")