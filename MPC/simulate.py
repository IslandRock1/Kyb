from time import perf_counter
import numpy as np
from linear_mpc import LinearMPC

def getModelWrist():
    A = np.array([
        [np.float64(0.9999884770328276), np.float64(-0.010177903510149211)],
        [np.float64(0.000292092434483153), np.float64(0.838127333943948)],
    ])

    B = np.array([
        [np.float64(-1.078485618445664e-06)],
        [np.float64(1.100257714200534e-05)],
    ])

    C = np.array([
        [np.float64(-846.2697104702287), np.float64(-105.48431381639631)],
    ])

    D = np.array([
        [np.float64(0.0)],
    ])

    return A, B, C, D

def simulate_system():
    A, B, C, D = getModelWrist()

    # MPC cost matrices
    Q = np.diag([1.0, 1.0])
    R = np.diag([1.0])

    # Initial state
    x0 = np.array([[0.0],
                   [0.0]])

    mpc_system = LinearMPC(A, B, C, D, Q, R, n_horizon=20, t_step=0.01)
    mpc_system.init_controller(x0)

    x_current = x0
    x_log, u_log = [], []
    N = 60

    x_curr = x0
    for _ in range(2):
        x_curr = mpc_system.sim.make_step(np.array([[255.0]]))
    # for _ in range(1):
    #     x_curr = mpc_system.sim.make_step(np.array([[0.0]]))

    print()
    print(f"x = np.array([[{x_curr[0,0]}, {x_curr[1,0]}]])")
    print(a0 := C @ x_curr)

    x_curr = mpc_system.sim.make_step(np.array([[0.0]]))
    print(f"np.array([[{x_curr[0,0]}, {x_curr[1,0]}]])")
    print(a1 := C @ x_curr)

    print(f"da: {a1 - a0}, dt: 0.01 => {(a1 - a0) / 0.01}")
    print(f"angle_vel = {(a1 - a0) / 0.01}")

    exit()

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
    time = np.arange(N) * 0.01

    LinearMPC.plot_results(time, x_log, u_log, y_log, Q, R)

if __name__ == "__main__":
    simulate_system()
