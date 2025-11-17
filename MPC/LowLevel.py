
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

def mpc_optimize(A, B, Q, R, P, x0, N, u_bounds=None):
    """
    Solve MPC problem for linear system using scipy.optimize.

    Args:
        A (ndarray): State matrix (n x n)
        B (ndarray): Input matrix (n x m)
        Q (ndarray): State weighting matrix (n x n)
        R (ndarray): Input weighting matrix (m x m)
        P (ndarray): Terminal state weighting (n x n)
        x0 (ndarray): Current state (n,)
        N (int): Horizon length
        u_bounds (list of tuples): Optional bounds for each input: [(umin, umax), ...] length = m

    Returns:
        u0_opt (ndarray): First optimal control input (m,)
        U_opt (ndarray): Full optimal control sequence (N x m)
    """
    n = A.shape[0]  # state dimension
    m = B.shape[1]  # input dimension

    # Flattened optimization variable: all inputs over horizon
    # U = [u0, u1, ..., u_{N-1}]
    def objective(U_flat):
        U = U_flat.reshape(N, m)
        x = x0.copy()
        cost = 0.0
        for k in range(N):
            u = U[k]
            cost += x.T @ Q @ x + u.T @ R @ u
            x = A @ x + B @ u
        # Terminal cost
        cost += x.T @ P @ x
        return cost

    # Initial guess: zeros
    U0 = np.zeros(N * m)

    # Setup bounds if provided
    bounds = None
    if u_bounds is not None:
        bounds = u_bounds * N  # repeat for each step in horizon

    # Solve optimization
    res = minimize(objective, U0, bounds=bounds, method='SLSQP')

    if not res.success:
        print("Warning: Optimization did not converge:", res.message)

    U_opt = res.x.reshape(N, m)
    u0_opt = U_opt[0]

    return u0_opt, U_opt

def main():
    # System matrices
    A = np.array([[1.0, 0.1],
                  [0.0, 1.0]])
    B = np.array([[0.0],
                  [0.1]])
    C = np.array([[1.0, 0.0]])
    D = np.array([[0.0]])

    # MPC cost matrices
    Q = np.diag([1.0, 1.0])
    R = np.diag([1.0])
    P = np.eye(2)

    # Simulation parameters
    x0 = np.array([0.5, 0.0])
    N = 20          # MPC horizon
    Tsim = 6.0      # total simulation time
    dt = 0.1        # timestep
    steps = int(Tsim/dt)

    # Logging
    n = A.shape[0]
    m = B.shape[1]
    p = C.shape[0]

    x_log = np.zeros((steps, n))
    u_log = np.zeros((steps, m))
    y_log = np.zeros((steps, p))
    time = np.linspace(0, Tsim, steps)

    x = x0.copy()

    for k in range(steps):
        # Get optimal input from MPC
        u0_opt, U_opt = mpc_optimize(A, B, Q, R, P, x, N)
        u = u0_opt

        # Log
        x_log[k] = x
        u_log[k] = u
        y_log[k] = (C @ x + D @ u).flatten()

        # Step system
        x = (A @ x + B @ u).flatten()

    # Plot results similar to your 3-panel plot
    fig, axs = plt.subplots(3, 1, figsize=(8, 6))
    fig.suptitle(f"Q: {Q[0,0]}, {Q[1,1]} | R: {R[0,0]}", fontsize=14)

    axs[0].plot(time, x_log[:, 0], label='x1')
    axs[0].plot(time, x_log[:, 1], label='x2')
    axs[0].legend()
    axs[0].set_ylabel('States')

    axs[1].plot(time, u_log, label='u')
    axs[1].legend()
    axs[1].set_ylabel('Input')

    axs[2].plot(time, y_log[:, 0], label='y')
    axs[2].legend()
    axs[2].set_ylabel('Output')
    axs[2].set_xlabel('Time [s]')

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.savefig(f"Q{Q[0,0]}_{Q[1,1]}_R{R[0,0]}.svg")
    plt.show()

if __name__ == "__main__":
    main()