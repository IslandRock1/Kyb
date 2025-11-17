import numpy as np
from linear_mpc import LinearMPC

def run_real_system(read_state_func, send_control_func, N=100, t_step=0.1):
    """
    read_state_func: function that returns current x as np.array([[x1],[x2]])
    send_control_func: function that applies u to actuators
    """
    # System matrices (needed for MPC even if real)
    A = np.array([[1.0, 0.1],
                  [0.0, 1.0]])
    B = np.array([[0.0],
                  [0.1]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    Q = np.diag([10000.0, 1000.0])
    R = np.diag([1.0])

    x0 = read_state_func()
    mpc_system = LinearMPC(A, B, C, D, Q, R, n_horizon=20, t_step=t_step)
    mpc_system.init_controller(x0)

    for _ in range(N):
        x_current = read_state_func()
        u0 = mpc_system.step(x_current)
        send_control_func(u0)
