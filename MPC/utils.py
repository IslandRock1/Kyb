
import numpy as np
import matplotlib.pyplot as plt

def get_model_shoulder():
    A = np.array([
        [np.float64(0.9999895681021433), np.float64(-0.012167569873969613)],
        [np.float64(0.00016549259115025193), np.float64(0.8689026919480014)],
    ])

    B = np.array([
        [np.float64(1.2772066767538635e-06)],
        [np.float64(-9.754379850423571e-06)],
    ])

    C = np.array([
        [np.float64(-859.987707055645), np.float64(-138.63095027350832)],
    ])

    D = np.array([
        [np.float64(0.0)],
    ])

    return A, B, C, D

def get_model_wrist():
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

def getCompleteModel():

    AShoulder, BShoulder, CShoulder, DShoulder = get_model_shoulder()
    AWrist, BWrist, CWrist, DWrist = get_model_wrist()

    A = np.zeros((4, 4), dtype=np.float64)
    A[0:2,0:2] = AShoulder
    A[2:,2:] = AWrist

    B = np.zeros((4, 2), dtype=np.float64)
    B[0:2,0] = BShoulder.flatten()
    B[2:,1] = BWrist.flatten()

    C = np.zeros((2, 4))
    C[0,0:2] = CShoulder
    C[1,2:] = CWrist

    D = np.array([[0.0]])

    return A, B, C, D

def plot_results(time, x_log, u_log, y_log, y1_log, Q, R, title = None):
    fig, axs = plt.subplots(3, 1, figsize=(8, 6))

    if (title is None):
        fig.suptitle(f"Output Cost Q: {Q[0,0]},{Q[1,1]} | Input Cost R: {R[0,0]}", fontsize=14)
    else:
        fig.suptitle(title)

    axs[0].plot(time, x_log[:, 0], label='x1')
    axs[0].plot(time, x_log[:, 1], label='x2')
    axs[0].plot(time, x_log[:, 2], label='x3')
    axs[0].plot(time, x_log[:, 3], label='x4')
    axs[0].legend()
    axs[0].set_ylabel('States')
    axs[0].grid(True)

    axs[1].plot(time, u_log[:, 0] * 12.0 / 255.0, label='u1')
    axs[1].plot(time, u_log[:, 1] * 12.0 / 255.0, label='u2')
    axs[1].legend()
    axs[1].set_ylabel('Input [volt]')
    axs[1].grid(True)

    axs[2].plot(time, y_log[:,0], label='angle1')
    axs[2].plot(time, y_log[:,1], label='angle2')

    # axs[2].plot(time, y1_log, label = 'angle velocity')
    axs[2].legend()
    axs[2].set_ylabel('Output [deg v deg/s]')
    axs[2].set_xlabel('Time [s]')
    axs[2].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    if (title is None): plt.savefig(f"MPC/Plots/Q{Q[0,0]}_R{R[0,0]}.svg")
    else: plt.savefig("MPC/Plots/" + title + ".svg")

    plt.show()