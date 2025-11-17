from time import perf_counter
import numpy as np
from linear_mpc import LinearMPC

def simulate_system():
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

    # Initial state
    x0 = np.array([[0.5],
                   [0.0]])

    mpc_system = LinearMPC(A, B, C, D, Q, R, n_horizon=20, t_step=0.1)
    mpc_system.init_controller(x0)

    x_current = x0
    x_log, u_log = [], []
    N = 60

    sum_time = 0.0
    tot_steps = 0
    for _ in range(N):
        t0 = perf_counter()
        u0 = mpc_system.step(x_current)
        t1 = perf_counter()
        x_current = mpc_system.sim.make_step(u0)
        x_log.append(x_current.flatten())
        u_log.append(float(u0))

        sum_time += t1 - t0
        tot_steps += 1

    print(f"Total time: {sum_time} | Num steps: {tot_steps} | Avg time: {1000.0 * sum_time / tot_steps} ms.")

    x_log = np.array(x_log)
    u_log = np.array(u_log)
    y_log = (C @ x_log.T).T
    time = np.arange(N) * 0.1

    LinearMPC.plot_results(time, x_log, u_log, y_log, Q, R)

if __name__ == "__main__":
    simulate_system()
