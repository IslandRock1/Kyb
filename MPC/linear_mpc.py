import numpy as np
import casadi as ca
import do_mpc

class LinearMPC:
    def __init__(self, A, B, C, D, Q, R, n_horizon, t_step):
        self.A, self.B, self.C, self.D = A, B, C, D
        self.nx, self.nu = A.shape[0], B.shape[1]
        self.Q, self.R = Q, R
        self.n_horizon = n_horizon
        self.t_step = t_step

        self._build_model()
        self._setup_mpc()
        self._setup_simulator()

    def _build_model(self):
        self.model = do_mpc.model.Model("discrete")
        self.x = self.model.set_variable(var_type='_x', var_name='x', shape=(self.nx, 1))
        self.u = self.model.set_variable(var_type='_u', var_name='u', shape=(self.nu, 1))

        x_next = ca.mtimes(ca.SX(self.A), self.x) + ca.mtimes(ca.SX(self.B), self.u)
        self.model.set_rhs('x', x_next)

        y = ca.mtimes(self.C, self.x) + ca.mtimes(self.D, self.u)
        self.model.set_expression('y', y)

        self.model.setup()

    def _setup_mpc(self):
        self.mpc = do_mpc.controller.MPC(self.model)
        self.mpc.set_param(
            n_horizon=self.n_horizon,
            t_step=self.t_step,
            state_discretization='discrete',
            store_full_solution=True
        )

        lterm = ca.mtimes([self.x.T, self.Q, self.x]) + ca.mtimes([self.u.T, self.R, self.u])
        mterm = ca.mtimes([self.x.T, self.Q, self.x])
        self.mpc.set_objective(lterm=lterm, mterm=mterm)
        self.mpc.set_rterm(ca.SX(self.R))

        self.mpc.bounds['lower', '_u', 'u'] = -1.0
        self.mpc.bounds['upper', '_u', 'u'] = 1.0

        self.mpc.setup()

    def _setup_simulator(self):
        self.sim = do_mpc.simulator.Simulator(self.model)
        self.sim.set_param(t_step=self.t_step)
        self.sim.setup()

    def init_controller(self, x0):
        """Initialize MPC for either simulation or real system."""
        self.mpc.x0 = x0
        self.mpc.u0 = np.zeros((self.nu, 1))
        self.mpc.set_initial_guess()

    def step(self, x_current):
        """Compute the control input for a given state."""
        return self.mpc.make_step(x_current)

    @staticmethod
    def plot_results(time, x_log, u_log, y_log, Q, R):
        import matplotlib.pyplot as plt
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
