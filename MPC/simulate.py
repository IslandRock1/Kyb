from time import perf_counter
import numpy as np
from linear_mpc import LinearMPC
from PID import  PID

from utils import getCompleteModel, plot_results
print() # Fuck CLion

pid0 = PID(40.0, 0.0, 0.0, 0.0, (-255.0, 255.0), 0.01)
pid1 = PID(40.0, 0.0, 0.0, 0.0, (-255.0, 255.0), 0.01)
mode = "PID"

def get_x0(num_steps_active, num_steps_passive):
    """

      0,   0 =>  0.000,  0.000
     10,   0 =>  1.658,  1.540
    100,   0 => 43.674, 35.410
    100,  10 => 46.791, 37.680
    100, 100 => 47.697, 38.048
      0, 100 =>  0.000,  0.000

    """


    A, B, C, D = getCompleteModel()
    u = np.array([[-255.0], [255.0]])
    x0 = np.zeros((4, 1))
    out = C @ x0

    num_active = 0
    num_passive = 0

    for num_active in range(num_steps_active):
        print(f"{out[0, 0]:.3f}, {out[1,0]:.3f}")
        x0 = A @ x0 + B @ u
        out = C @ x0

    for num_passive in range(num_steps_passive):
        print(f"{out[0, 0]:.3f}, {out[1,0]:.3f}")
        x0 = A @ x0
        out = C @ x0

    print(f"{out[0, 0]:.3f}, {out[1,0]:.3f}")

    print()
    print(out)
    return x0

def simulate_system():
    A, B, C, D = getCompleteModel()

    # MPC cost matrices
    Q = np.diag([1000000.0, 1000000.0, 10000.0, 10000.0])
    R = np.diag([0.000001, 0.000001])

    # Initial state
    x0 = get_x0(200, 1000) # Shoulder: 93 deg | Wrist: 74 deg
    initialState = C @ x0
    # print(f"Initial state: {initialState[0, 0]:.3f}, {initialState[1,0]:.3f}")

    mpc_system = LinearMPC(A, B, C, D, Q, R, n_horizon=20, t_step=0.01)
    mpc_system.init_controller(x0)

    x_current = x0
    x_log, u_log = [], []
    N = 500

    sum_time = 0.0
    tot_steps = 0
    for i in range(1, N + 1):
        print(f"{i}/{N} => {100.0 * i / N}%")

        t0 = perf_counter()
        if (mode == "PID"):
            outputs = (C @ x_current)
            u_from_pid0 = pid0.update(outputs[0,0])
            u_from_pid1 = pid1.update(outputs[2,0])
            u0 = np.array([[u_from_pid0], [u_from_pid1]])
        else:
            u0 = mpc_system.step(x_current)
        t1 = perf_counter()

        x_current = mpc_system.sim.make_step(u0)

        x_log.append(x_current.flatten())
        u_log.append(u0.flatten())

        sum_time += t1 - t0
        tot_steps += 1

    print()
    print(f"Total time: {sum_time} | Num steps: {tot_steps} | Avg time: {1000.0 * sum_time / tot_steps} ms.")

    x_log = np.array(x_log)
    u_log = np.array(u_log)
    y_log = (C @ x_log.T).T
    y1_log = (C @ x_log.T).T
    time = np.arange(N) * 0.01

    if (mode == "PID"):
        plot_results(time, x_log, u_log, y_log, y1_log, Q, R, f"PID")
    else:
        plot_results(time, x_log, u_log, y_log, y1_log, Q, R, "MPC")

if __name__ == "__main__":
    simulate_system()
