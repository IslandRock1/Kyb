"""

This file reads a CSV file with collected data and then generates 3 plots:
1. Forces vs time
2. Torque vs time
3. Raw sensor values vs time

"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

# Read the CSV file
df = pd.read_csv(r'PythonCode/DATA/data.csv')

# Convert Timestamp to relative time (seconds since start)
df['Relative_Time'] = df['Timestamp'] - df['Timestamp'].min()

# Plot 1: Forces (Fx, Fy, Fz) vs Time
plt.figure(figsize=(10, 6))
for col in ['Fx', 'Fy', 'Fz']:
    plt.plot(df['Relative_Time'], df[col], label=col)
plt.xlabel('Time (seconds)')
plt.ylabel('Force (N)')
plt.title('Forces vs Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(f'PythonCode/Plots/forces_plot.png')
plt.close()

# Plot 2: Moments (Mx, My, Mz) vs Time
plt.figure(figsize=(10, 6))
for col in ['Mx', 'My', 'Mz']:
    plt.plot(df['Relative_Time'], df[col], label=col)
plt.xlabel('Time (seconds)')
plt.ylabel('Moment (Nm)')
plt.title('Moments vs Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.savefig(f'PythonCode/Plots/moments_plot.png')
plt.close()

# Plot 3: Sensor Values (s0-s3 and s4-s7) vs Time (Scatter Plot with Subplots)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
# Subplot 1: s0 to s3
for col in ['s0', 's1', 's2', 's3']:
    ax1.scatter(df['Relative_Time'], df[col], label=col, alpha=0.5, s=1)
ax1.set_ylabel('Sensor Value')
ax1.set_title('Sensor Values s0-s3 vs Time')
ax1.legend(loc='best', ncol=2, markerscale=10)
ax1.grid(True)

# Subplot 2: s4 to s7
for col in ['s4', 's5', 's6', 's7']:
    ax2.scatter(df['Relative_Time'], df[col], label=col, alpha=0.5, s=1)
ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Sensor Value')
ax2.set_title('Sensor Values s4-s7 vs Time')
ax2.legend(loc='best', ncol=2, markerscale=10)
ax2.grid(True)

plt.tight_layout()
plt.savefig(f'PythonCode/Plots/sensor_values_plot.png')
plt.close()