from time import perf_counter
import numpy as np
from linear_mpc import LinearMPC
from PID import  PID

from utils import getCompleteModel, plot_results
print() # Fuck CLion

pid0 = PID(40.0, 0.0, 0.0, 0.0, (-255.0, 255.0), 0.01)
pid1 = PID(40.0, 0.0, 0.0, 0.0, (-255.0, 255.0), 0.01)

def get_x0(num_steps_active, num_steps_passive):
    """

      0,   0 =>  0.0, 0.0, 0.0, 0.0
     10,   0 =>  0.0006330490825515031, -0.007061460517633658, 0.036716916022537405, -0.2053147420398154
    100,   0 => 43.674, 35.410
    100,  10 => 46.791, 37.680
    100, 100 => 47.697, 38.048
      0, 100 =>  0.0, 0.0, 0.0, 0.0

    """


    A, B, C, D = getCompleteModel()
    u = np.array([[255.0], [255.0]])
    x0 = np.zeros((4, 1))
    out = C @ x0

    num_active = 0
    num_passive = 0

    for num_active in range(num_steps_active):
        # print(f"{out[0, 0]:.3f}, {out[1,0]:.3f}")
        x0 = A @ x0 + B @ u
        out = C @ x0

    for num_passive in range(num_steps_passive):
        # print(f"{out[0, 0]:.3f}, {out[1,0]:.3f}")
        x0 = A @ x0
        out = C @ x0

    print(f"{out[0, 0]:.3f}, {out[1,0]:.3f}, {out[2,0]:.3f}, {out[3,0]:.3f}")
    return x0

def simulate_system():
    A, B, C, D = getCompleteModel()

    qv = 5000
    Qy = np.diag([qv, 0, qv, 0])
    Q = C.T @ Qy @ C
    print(f"Q: {Q}")
    R = np.diag([1.0, 1.0])
    mode = "MPC"

    # Initial state
    x0 = get_x0(200, 1000) # Shoulder: 92 deg | Wrist: 75 deg
    initialState = C @ x0
    print(f"Initial state: {initialState[0, 0]:.3f}, {initialState[2,0]:.3f}")


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
    time = np.arange(N) * 0.01

    if (mode == "PID"):
        tuning = (pid0, pid1)
    elif (mode == "MPC"):
        tuning = (Q, R)
    else:
        raise ValueError("What??")

    plot_results(time, x_log, u_log, y_log, tuning)

if __name__ == "__main__":
    simulate_system()
