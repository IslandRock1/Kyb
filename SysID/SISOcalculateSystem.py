import pandas as pd
import numpy as np
import control as ct
from sippy_unipi import system_identification as sysid
import matplotlib.pyplot as plt

training_files = [
    r"PythonCode\DATA\wrist_sysid_1.csv",
    r"PythonCode\DATA\wrist_sysid_2.csv",
    r"PythonCode\DATA\wrist_sysid_3.csv",
    r"PythonCode\DATA\wrist_sysid_4.csv",
]

input_training_list = []
output_training_list = []
timestamp_trianing_list = []

for f in training_files:
    data = pd.read_csv(f)
    input_training_list.append(data['gain'].values)
    output_training_list.append(data['position'].values)
    timestamp_trianing_list.append(data['timestamp'].values)

input_training = np.concatenate(input_training_list)
output_training = np.concatenate(output_training_list)
timestamp_training = np.concatenate(timestamp_trianing_list)
sample_time = np.mean([np.mean(np.diff(t)) for t in timestamp_trianing_list])




model = sysid(output_training, input_training, 'N4SID', tsample=sample_time, SS_fixed_order=2)

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

#creating system
state_space_system = ct.StateSpace(model.A, model.B, model.C, model.D, sample_time)

validation_file = r"PythonCode\DATA\wrist_sysid_5.csv"
validation_data = pd.read_csv(validation_file)
input_validation = validation_data['gain'].values
output_validation = validation_data['position'].values
timestamp_validation = validation_data['timestamp'].values

n_samples_validation = len(input_validation)
timestamp_uniform_validation = np.arange(n_samples_validation) * sample_time


#checking response
_, output_predicted_validation = ct.forced_response(state_space_system, T=timestamp_uniform_validation, U=input_validation)

rmse = np.sqrt(np.mean((output_validation - output_predicted_validation)**2))
print(f"RMSE = {rmse:.4f} degrees")

plt.figure(figsize=(10,4))
plt.plot(timestamp_uniform_validation, output_validation, label='Actual')
plt.plot(timestamp_uniform_validation, output_predicted_validation, label='Predicted', linestyle='--')
plt.xlabel('Time [s]')
plt.ylabel('Position [deg]')
plt.title(f'Validation: Actual vs Predicted (Root Mean Square Error = {rmse:.2f}°)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()