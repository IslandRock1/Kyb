"""

This file does the same as "4_validation.py", but including quadratic terms.

It uses the raw sensor values from validation data (s_0 to s_7),
to compute the estimated wrench using the calibration matrices C and L: W_est = C + LS + QS^2.

It also reads the wrench values from the validation data (W_ref).

Then the error is computed by: Error = W_est - W_ref

Finally, the error is plotted in three plots:
1. Force error vs time for each [Fx, Fy, Fz]
2. Torque error vs time for each [Mx, My, Mz]
3. Histogram of error for each wrench value [Fx, Fy, Fz, Mx, My, Mz]

"""

import numpy as np
import sys
import pandas as pd
import matplotlib.pyplot as plt

print('Starting...')

# Function to compute the wrench W = [Fx, Fy, Fz, Mx, My, Mz], based on C, L, Q and S (W=C+LS+QS^2)
def compute_wrench(C, L, Q, S):
    W = np.zeros(6)
    S = np.array(S)
    # Linear terms: C + LS
    W += C.flatten() + L @ S
    # Quadratic terms: QS^2
    quad_terms = np.array([S[i] * S[j] for i in range(len(S)) for j in range(i, len(S))])
    W += Q @ quad_terms
    return W.tolist()

try:
    n_sensors = 8
    overload_lower = 50
    overload_upper = 950

    # Load calibrated C and L
    model_name = "lasso"
    # df = pd.read_csv(r'PythonCode/DATA/params_linearregression_quadratic.csv')
    df = pd.read_csv(fr'PythonCode/DATA/params_{model_name}_quadratic.csv')
    C = df[['C']].values # Shape: (6, 1)
    L = df[['L_s0','L_s1','L_s2','L_s3','L_s4','L_s5','L_s6','L_s7']].values # Shape: (6, 8)
    Q_cols = [f'Q_s{i}s{j}' for i in range(n_sensors) for j in range(i, n_sensors)]
    Q = df[Q_cols].values  # Shape: (6, 36)

    # Read the validation data
    dfv = pd.read_csv(r'PythonCode/DATA/val_data.csv')

    # Initialize lists to store errors
    errors = []

    # Process each row of validation data
    for index, row in dfv.iterrows():
        # Get sensor values s0-s7
        s = [row[f's{i}'] for i in range(n_sensors)]

        # Check for overload
        #skip_row = False
        #for i in range(n_sensors):
        #    if (s[i] < overload_lower) or (s[i] > overload_upper):
        #        print(f"Force overload channel {i} at row {index}")
        #        skip_row = True
        #        break
        #if skip_row:
        #    continue

        # Compute estimated wrench
        W_est = compute_wrench(C, L, Q, s)

        # Get real wrench values
        W_real = [row['Fx'], row['Fy'], row['Fz'], row['Mx'], row['My'], row['Mz']]

        # Compute error
        error = [est - real for est, real in zip(W_est, W_real)]

        # Store error with row information
        error_row = {
            'row_index': index,
            'Fx_error': error[0],
            'Fy_error': error[1],
            'Fz_error': error[2],
            'Mx_error': error[3],
            'My_error': error[4],
            'Mz_error': error[5]
        }
        errors.append(error_row)

    # Create DataFrame from errors and save to CSV
    error_df = pd.DataFrame(errors)
    error_df.to_csv(f'PythonCode/Plots/error_val_{model_name}_quadratic.csv', index=False)

    # Plot 1: Forces and Moments
    plt.figure(figsize=(12, 5))
    # Subplot 1.1: Forces
    plt.subplot(1, 2, 1)
    plt.scatter(error_df['row_index'], error_df['Fx_error'], label='Fx error', s=1)
    plt.scatter(error_df['row_index'], error_df['Fy_error'], label='Fy error', s=1)
    plt.scatter(error_df['row_index'], error_df['Fz_error'], label='Fz error', s=1)
    plt.xlabel('Sample Index')
    plt.ylabel('Force Error (N)')
    plt.title('Force Errors')
    plt.legend(markerscale=10)
    plt.axhline(y=0, color='k', linewidth=1, linestyle='-')
    plt.grid(True)

    # Subplot 1.2: Moments
    plt.subplot(1, 2, 2)
    plt.scatter(error_df['row_index'], error_df['Mx_error'], label='Mx error', s=1)
    plt.scatter(error_df['row_index'], error_df['My_error'], label='My error', s=1)
    plt.scatter(error_df['row_index'], error_df['Mz_error'], label='Mz error', s=1)
    plt.xlabel('Sample Index')
    plt.ylabel('Moment Error (Nm)')
    plt.title('Moment Errors')
    plt.legend(markerscale=10)
    plt.axhline(y=0, color='k', linewidth=1, linestyle='-')
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(f'PythonCode/Plots/error_val_{model_name}_quadratic.png')
    plt.close()

    # Plot 2: Error Histograms
    plt.figure(figsize=(15, 10))
    plt.subplot(2, 3, 1)
    plt.hist(error_df['Fx_error'], bins=30, color='red', edgecolor='black')
    plt.xlabel('Fx Error (N)')
    plt.ylabel('Count')
    plt.title('Fx Error Distribution')
    plt.grid(True)
    xlim = max(abs(min(error_df['Fx_error'])), abs(max(error_df['Fx_error'])))
    plt.xlim(-xlim, xlim)
    plt.axvline(x=0, color='k', linewidth=2, linestyle='-')

    plt.subplot(2, 3, 2)
    plt.hist(error_df['Fy_error'], bins=30, color='green', edgecolor='black')
    plt.xlabel('Fy Error (N)')
    plt.ylabel('Count')
    plt.title('Fy Error Distribution')
    plt.grid(True)
    xlim = max(abs(min(error_df['Fy_error'])), abs(max(error_df['Fy_error'])))
    plt.xlim(-xlim, xlim)
    plt.axvline(x=0, color='k', linewidth=2, linestyle='-')

    plt.subplot(2, 3, 3)
    plt.hist(error_df['Fz_error'], bins=30, color='blue', edgecolor='black')
    plt.xlabel('Fz Error (N)')
    plt.ylabel('Count')
    plt.title('Fz Error Distribution')
    plt.grid(True)
    xlim = max(abs(min(error_df['Fz_error'])), abs(max(error_df['Fz_error'])))
    plt.xlim(-xlim, xlim)
    plt.axvline(x=0, color='k', linewidth=2, linestyle='-')

    plt.subplot(2, 3, 4)
    plt.hist(error_df['Mx_error'], bins=30, color='orange', edgecolor='black')
    plt.xlabel('Mx Error (Nm)')
    plt.ylabel('Count')
    plt.title('Mx Error Distribution')
    plt.grid(True)
    xlim = max(abs(min(error_df['Mx_error'])), abs(max(error_df['Mx_error'])))
    plt.xlim(-xlim, xlim)
    plt.axvline(x=0, color='k', linewidth=2, linestyle='-')

    plt.subplot(2, 3, 5)
    plt.hist(error_df['My_error'], bins=30, color='purple', edgecolor='black')
    plt.xlabel('My Error (Nm)')
    plt.ylabel('Count')
    plt.title('My Error Distribution')
    plt.grid(True)
    xlim = max(abs(min(error_df['My_error'])), abs(max(error_df['My_error'])))
    plt.xlim(-xlim, xlim)
    plt.axvline(x=0, color='k', linewidth=2, linestyle='-')

    plt.subplot(2, 3, 6)
    plt.hist(error_df['Mz_error'], bins=30, color='cyan', edgecolor='black')
    plt.xlabel('Mz Error (Nm)')
    plt.ylabel('Count')
    plt.title('Mz Error Distribution')
    plt.grid(True)
    xlim = max(abs(min(error_df['Mz_error'])), abs(max(error_df['Mz_error'])))
    plt.xlim(-xlim, xlim)
    plt.axvline(x=0, color='k', linewidth=2, linestyle='-')

    plt.tight_layout()
    plt.savefig(f'PythonCode/Plots/error_dist_val_{model_name}_quadratic.png')
    plt.close()


except KeyboardInterrupt:
    # ctrl-C abort handling
    print('Stopped.')
except Exception as exp:
    print("Exception. Something went wrong.")
    print(f"Exception: {exp}")
    sys.exit(1)
finally:
    print('Finished.')


