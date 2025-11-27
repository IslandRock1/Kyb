import pandas as pd
import numpy as np
import control as ct
import matplotlib.pyplot as plt
from sippy_unipi import system_identification as sysid

training_files = [
    r"SysID/DATA/MIMO_full.csv",
]

input_training_list = []
output_training_list = []
timestamp_training_list = []

for f in training_files:
    data = pd.read_csv(f)
    t = data["timestamp"].values
    sh_pos = data["shoulder_position"].values
    wr_pos = data["wrist_position"].values
    sh_gain = data["shoulder_gain"].values
    wr_gain = data["wrist_gain"].values
    sample_time_local = np.mean(np.diff(t))
    sh_vel = np.gradient(sh_pos, sample_time_local)
    wr_vel = np.gradient(wr_pos, sample_time_local)
    input_training_list.append(np.vstack([sh_gain, wr_gain]).T)
    output_training_list.append(np.vstack([sh_pos, sh_vel, wr_pos, wr_vel]).T)
    timestamp_training_list.append(t)

inputs_training = np.concatenate(input_training_list)
outputs_training = np.concatenate(output_training_list)
timestamps_training = np.concatenate(timestamp_training_list)

print(type(inputs_training))

sample_time = np.mean([np.mean(np.diff(t)) for t in timestamp_training_list])

model = sysid(outputs_training, inputs_training, 'N4SID', tsample=sample_time, SS_fixed_order=4)

#this printing code is made by ChatGPT
print("A = np.array([")
for row in model.A:
    print("    " + str(list(row)) + ",")
print("])\n")

print("B = np.array([")
for row in model.B:
    print("    " + str(list(row)) + ",")
print("])\n")

print("C = np.array([")
for row in model.C:
    print("    " + str(list(row)) + ",")
print("])\n")

print("D = np.array([")
for row in model.D:
    print("    " + str(list(row)) + ",")
print("])\n")

state_space_system = ct.StateSpace(model.A, model.B, model.C, model.D, sample_time)

validation_file = r"SysID/DATA/validation_2.csv"
validation_data = pd.read_csv(validation_file)

tv = validation_data["timestamp"].values
sh_pos_v = validation_data["shoulder_position"].values
wr_pos_v = validation_data["wrist_position"].values
sh_gain_v = validation_data["shoulder_gain"].values
wr_gain_v = validation_data["wrist_gain"].values

sh_vel_v = np.gradient(sh_pos_v, sample_time)
wr_vel_v = np.gradient(wr_pos_v, sample_time)

input_validation = np.vstack([sh_gain_v, wr_gain_v]).T
output_validation = np.vstack([sh_pos_v, sh_vel_v, wr_pos_v, wr_vel_v]).T

n_samples_validation = len(input_validation)
timestamp_uniform_validation = np.arange(n_samples_validation) * sample_time

_, output_predicted_validation = ct.forced_response(state_space_system, T=timestamp_uniform_validation, U=input_validation.T)


y_mean = np.mean(output_validation, axis=0)
fit = 100 * (1 - np.linalg.norm(output_validation - output_predicted_validation.T, axis=0) /
             np.linalg.norm(output_validation - y_mean, axis=0))

#This graphing is made by ChatGPT
labels = ["Shoulder Pos", "Shoulder Vel", "Wrist Pos", "Wrist Vel"]
plt.figure(figsize=(12, 10))
for i in range(4):
    plt.subplot(4, 1, i+1)
    plt.plot(timestamp_uniform_validation, output_validation[:, i], label="Actual")
    plt.plot(timestamp_uniform_validation, output_predicted_validation[i], "--", label="Predicted")

    plt.title(f"{labels[i]} (FIT={fit[i]:.1f}%)")

    plt.grid(True)
    plt.legend()

plt.tight_layout()
plt.show()