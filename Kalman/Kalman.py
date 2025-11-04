# Kalman filter based on "A Linear Discrete Kalman Filter to Estimate the Contact Wrench of an
# Unknown Robot End Effector" by Skrede (2024)

import numpy as np

def skew(v):
    """Returns skew-symmetric matrix of a 3-vector."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

class ContactWrenchKalmanFilter:
    def __init__(self, m, r_s, Q, Rf, Ra, init_state=None, init_cov=None):
        """Initializes KF.

        Args:
            m: float, estimated mass of end effector.
            r_s: array-like, shape (3,), estimated mass center in s frame.
            Q: array-like, shape (9,9), process covariance.
            Rf: array-like, shape (6,6), meas. cov. for FTS.
            Ra: array-like, shape (3,3), meas. cov. for IMU.
            init_state: (optional) array-like, shape (9,), initial state.
            init_cov: (optional) array-like, shape (9,9), initial cov matrix.
        """
        self.m = m
        self.r_s_vector = np.array(r_s)
        self.r_s = skew(self.r_s_vector)
        self.Q = Q
        self.Rf = Rf
        self.Ra = Ra
        self.x = np.zeros(9) if init_state is None else np.array(init_state)  # [a(3), F(3), T(3)]
        self.P = np.eye(9) if init_cov is None else np.array(init_cov)

        # State transition (identity)
        self.A = np.eye(9)                                           # (9,9)

        # Measurement matrices
        self.Hf = np.zeros((6, 9))
        self.Hf[0:3, 3:6] = np.eye(3)      # F
        self.Hf[3:6, 6:9] = np.eye(3)      # T

        self.Ha = np.zeros((3, 9))
        self.Ha[0:3, 0:3] = np.eye(3)      # a

    def predict(self, u, B):
        # Prediction step
        self.x = self.A @ self.x + B @ u
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update_FTS(self, zf):
        # Correction step for FTS
        S = self.Hf @ self.P @ self.Hf.T + self.Rf
        K = self.P @ self.Hf.T @ np.linalg.inv(S)
        y = zf - self.Hf @ self.x
        self.x = self.x + K @ y
        self.P = self.P - K @ self.Hf @ self.P

    def update_IMU(self, za):
        # Correction step for IMU
        S = self.Ha @ self.P @ self.Ha.T + self.Ra
        K = self.P @ self.Ha.T @ np.linalg.inv(S)
        y = za - self.Ha @ self.x
        self.x = self.x + K @ y
        self.P = self.P - K @ self.Ha @ self.P

    def contact_wrench_estimate(self):
        # Output equation (Skrede eq. 22): z_c = [m*I3 | I3 | 0; m*[r_s]_x | 0 | I3] x
        m, r_s = self.m, self.r_s
        # Build H_c (6x9)
        Hc = np.zeros((6, 9))
        # Forces
        Hc[0:3, 0:3] = m * np.eye(3)
        Hc[0:3, 3:6] = np.eye(3)
        # Torques
        Hc[3:6, 0:3] = m * r_s
        Hc[3:6, 6:9] = np.eye(3)
        return Hc @ self.x

# --- Example usage skeleton ---

# Assume m, r_s, Q, Rf, Ra set as in the paper for your system.
kf = ContactWrenchKalmanFilter(m=0.932, r_s=[0,0,0.044], Q=np.eye(9)*1e-3,
                               Rf=np.eye(6)*1e-2, Ra=np.eye(3)*1e-1)

while data_available:
    # Compute control input (u) and process matrix (B) as in text eq (15-16)
    # Requires current and past orientations for gravity compensation
    u = ... # (3,) from processed grav diff and scaling
    B = np.block([
        [np.eye(3)],
        [kf.m*np.eye(3)],
        [kf.m*kf.r_s]
    ]) # shape (9,3)

    kf.predict(u, B)                           # Predict step

    # Sensor update
    zf = ... # FTS measurement (6,)
    kf.update_FTS(zf)

    za = ... # IMU measurement (3,)
    kf.update_IMU(za)

    wrench = kf.contact_wrench_estimate()      # (6,)
    # wrench[0:3]: estimated force, wrench[3:6]: estimated torque
