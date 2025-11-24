import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from Kalman import ContactWrenchKalmanFilter

def analyze_and_plot(wrench_file, orient_file, plot_title):
    """
    Analyzes a dataset using the Kalman filter and plots the results.
    """
    print(f"\n--- Starting analysis for: {plot_title} ---")
    
    # load DataFrames
    try:
        df_wrench = pd.read_csv(wrench_file)
        df_orient = pd.read_csv(orient_file)
        print("Successfully loaded data files.")
    except FileNotFoundError as e:
        print(f"Error: {e}. Ensure data files are in the 'Kalman/DATA/' folder.")
        return

    # synchronize DataFrames by timestamp
    df_wrench = df_wrench.sort_values(by='t')
    df_orient = df_orient.sort_values(by='t')

    # merge by nearest previous timestamp
    df_merged = pd.merge_asof(
        df_wrench,
        df_orient,
        on='t',
        direction='backward'
    )

    # drop rows that couldn't match
    df_merged = df_merged.dropna()
    print(f"Data merged. Total rows: {len(df_merged)}")



    # Parameters from Skrede (2024), section IV-A

    # mass / center of mass (eq. 23)
    m = 0.932  # kg
    r_s = np.array([0, 0, 0.044])  # m

    # Noise covariance values (from example in Kalman.py)
    Q_val = 1e-3  # Process noise
    Rf_val = 1e-2 # Measurement noise for FTS

    kf = ContactWrenchKalmanFilter(
        m=m,
        r_s=r_s,
        Q=np.eye(9) * Q_val,
        Rf=np.eye(6) * Rf_val
    )
    print("Kalman Filter initialized (parameters from Skrede (2024)).")

    # gravitational acceleration in world frame [m/s^2]
    g_w = np.array([0, 0, -9.81])

    # frequencies from paper (eq. 24)  (used for scaling control input 'u')
    fr = 100.2  # Hz, robot controller
    ff = 698.3  # Hz, FTS
    fa = 254.3  # Hz, IMU
    freq_ratio = fr / (ff + fa)

    estimated_wrenches = []
    timestamps = []

    # values for the loop
    # extract the first orientation to initialize R_ws_previous
    first_row = df_merged.iloc[0]
    R_ws_previous = np.array([
        [first_row['r11'], first_row['r12'], first_row['r13']],
        [first_row['r21'], first_row['r22'], first_row['r23']],
        [first_row['r31'], first_row['r32'], first_row['r33']]
    ])

    # initial gravity vector in sensor frame
    g_s_previous = R_ws_previous.T @ g_w

    # define input matrix B (eq. 16 paper)
    # OBS: mass doesn't change
    B = np.block([
        [np.eye(3)],
        [m * np.eye(3)],
        [m * kf.r_s]  # kf.r_s is the skew-symmetric matrix of r_s
    ])

    print("Starting main loop...")

    for index, row in df_merged.iterrows():
        # current orientation, build rotation matrix R_ws_k
        R_ws_k = np.array([
            [row['r11'], row['r12'], row['r13']],
            [row['r21'], row['r22'], row['r23']],
            [row['r31'], row['r32'], row['r33']]
        ])

        # control input u, based on gravity change (eq. 15 paper)
        g_s_k = R_ws_k.T @ g_w
        delta_g_s = g_s_k - g_s_previous
        u = delta_g_s * freq_ratio

        # predict next state
        kf.predict(u, B)

        # current FTS measurement ('zf')
        zf = row[['fx', 'fy', 'fz', 'tx', 'ty', 'tz']].values

        # correct prediction with actual measurement
        kf.update_FTS(zf)

        # final contact wrench estimate
        wrench_estimate = kf.contact_wrench_estimate()
        estimated_wrenches.append(wrench_estimate)
        timestamps.append(row['t'])

        # update previous
        g_s_previous = g_s_k

    print("Processed all data points.")

    # Plots
    estimated_wrenches = np.array(estimated_wrenches)
    plt.figure(figsize=(12, 6))
    plt.plot(timestamps, df_merged['fz'], label='Original Fz (Støy)', alpha=0.5)
    plt.plot(timestamps, estimated_wrenches[:, 2], label='Estimert Fz (Kalman Filter)', linewidth=2)
    plt.title(plot_title)
    plt.xlabel("Tidsstempel")
    plt.ylabel("Kraft (N)")
    plt.legend()
    plt.grid(True)
    
    plt.show()

if __name__ == "__main__":
    DATA_DIR = "Kalman/DATA/"


    # 1. Baseline Test
    analyze_and_plot(
        wrench_file=DATA_DIR + "1-baseline_wrench.csv",
        orient_file=DATA_DIR + "1-baseline_orientations.csv",
        plot_title="Fz Estimate for Baseline Test"
    )

    # 2. Vibrations Test
    analyze_and_plot(
        wrench_file=DATA_DIR + "2-vibrations_wrench.csv",
        orient_file=DATA_DIR + "2-vibrations_orientations.csv",
        plot_title="Fz Estimate for Vibrations Test"
    )

    # 3. Vibrations and Contact Test
    analyze_and_plot(
        wrench_file=DATA_DIR + "3-vibrations-contact_wrench.csv",
        orient_file=DATA_DIR + "3-vibrations-contact_orientations.csv",
        plot_title="Fz Estimate for Vibrations and Contact Test"
    )
