
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

def getCompleteModelFromIndividual():

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

def getCompleteModel():
    A = np.array([[np.float64(0.9999976569526424),          np.float64(-9.160143138198587e-06), np.float64(-0.0004257708011442182), np.float64(-0.00035086091730667196)],
                  [np.float64(1.727268799697247e-05), np.float64(0.999990392439614), np.float64(-0.0005075781244622638), np.float64(0.000386153652274123)],
                  [np.float64(0.0012246538538164004), np.float64(-0.0022024204733846003), np.float64(0.896688079038486), np.float64(0.0022718901206865553)],
                  [np.float64(-0.0002572853227708653), np.float64(-0.0011503783795186694), np.float64(0.0027134622622971426), np.float64(0.8840435600426209)],
                  ])

    B = np.array([
        [np.float64(-5.130234624600828e-07), np.float64(-3.864981555405107e-07)],
        [np.float64(2.5524460134423797e-06), np.float64(-5.888735497623858e-09)],
        [np.float64(4.4891339011425385e-05), np.float64(6.843206808386824e-05)],
        [np.float64(7.320567113462283e-05), np.float64(-5.885263266275326e-05)],
    ])

    C = np.array([
        [np.float64(-530.0194089374477), np.float64(528.1787832390378),   np.float64(-14.799209025171988), np.float64(-14.392447777995834)],
        [np.float64(-0.4488393014090485), np.float64(5.211129355383186), np.float64(141.3909283438485), np.float64(190.667561330461)],
        [np.float64(-2215.242775047159), np.float64(-806.3831316057453), np.float64(-1.93158380335166), np.float64(14.06810683716181)],
        [np.float64(-1.3856567366757033), np.float64(1.4256328285072872), np.float64(151.47637397325803), np.float64(-108.46943621129427)],
    ])

    D = np.array([
        [np.float64(0.0), np.float64(0.0)],
        [np.float64(0.0), np.float64(0.0)],
        [np.float64(0.0), np.float64(0.0)],
        [np.float64(0.0), np.float64(0.0)],
    ])

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
    axs[2].plot(time, y_log[:,2], label='angle2')

    # axs[2].plot(time, y1_log, label = 'angle velocity')
    axs[2].legend()
    axs[2].set_ylabel('Output [deg v deg/s]')
    axs[2].set_xlabel('Time [s]')
    axs[2].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    if (title is None): plt.savefig(f"MPC/Plots/Q{Q[0,0]}_R{R[0,0]}.svg")
    else: plt.savefig("MPC/Plots/" + title + ".svg")

    plt.show()

