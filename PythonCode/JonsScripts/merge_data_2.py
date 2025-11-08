"""

This file reads all the collected data inside the same folder but in different CSV files,
and it merges everything into a single CSV file.

It can be used to filter out (or not) datapoints out of the bounds for the raw sensor values.

Finally, it separates the dataset into training and validation data,
using randomly chosen datapoints and separating into 80% and 20% respectively.

"""

import pandas as pd
import glob
import os

# Find all CSV files in the directory

directory = r"PythonCode/DATA/"
csv_files = glob.glob(os.path.join(directory, "*.csv"))
print(f"csv_files: {csv_files}")
# Expected header
expected_columns = ['Timestamp', 'Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz', 's0', 's1', 's2', 's3', 's4', 's5', 's6', 's7']

# Initialize an empty list to store DataFrames
dfs = []

n_sensors = 8
overload_lower = 50
overload_upper = 950

# Read the first CSV with header, others without
for i, file in enumerate(csv_files):
    try:
        df = pd.read_csv(file, dtype_backend='numpy_nullable')
        print(df)
        # Verify header
        if list(df.columns) != expected_columns:
            print(f"Warning: {file} has incorrect columns: {df.columns}")
            continue
        # Convert wrench and sensor columns to numeric, drop invalid rows
        wrench_cols = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']
        sensor_cols = ['s0', 's1', 's2', 's3', 's4', 's5', 's6', 's7']
        df[wrench_cols + sensor_cols] = df[wrench_cols + sensor_cols].apply(pd.to_numeric, errors='coerce')
        df = df.dropna(subset=wrench_cols + sensor_cols)

        # Filter out rows where any sensor value is out of bounds
        df = df[(df[sensor_cols] >= overload_lower).all(axis=1) & (df[sensor_cols] <= overload_upper).all(axis=1)]

        # For first file, keep header; for others, skip it
        if i > 0:
            df = df.iloc[1:]  # Skip header for non-first files
        dfs.append(df)
    except Exception as e:
        print(f"Error processing {file}: {e}")

# Concatenate valid DataFrames
if dfs:
    merged_df = pd.concat(dfs, ignore_index=True)
    # Save merged data
    merged_df.to_csv(os.path.join(directory, 'data.csv'), index=False)
    print(f"Merged {len(dfs)} files into 'data.csv' with {len(merged_df)} rows.")
    # Split data into train (80%) and validation (20%)
    train_df = merged_df.sample(frac=0.8, random_state=42)
    val_df = merged_df.drop(train_df.index)
    # Save split datasets
    train_df.to_csv(os.path.join(directory, 'train_data.csv'), index=False)
    val_df.to_csv(os.path.join(directory, 'val_data.csv'), index=False)
    print(f"Split into train_data.csv ({len(train_df)} rows) and val_data.csv ({len(val_df)} rows).")

else:
    print("No valid CSV files found.")