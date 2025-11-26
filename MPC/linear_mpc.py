import numpy as np
import casadi as ca
import do_mpc

class LinearMPC:
    def __init__(self, A, B, C, D, Q, R, n_horizon, t_step):
        self.A, self.B, self.C, self.D = A, B, C, D
        self.nx, self.nu, self.ny = A.shape[0], B.shape[1], C.shape[0]
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

        x_next = ca.mtimes(ca.DM(self.A), self.x) + ca.mtimes(ca.DM(self.B), self.u)
        self.model.set_rhs('x', x_next)

        y = ca.mtimes(ca.DM(self.C), self.x) + ca.mtimes(ca.DM(self.D), self.u)
        self.model.set_expression('y', y)

        self.model.setup()

    def _setup_mpc(self):
        self.mpc = do_mpc.controller.MPC(self.model)
        self.mpc.settings.supress_ipopt_output()
        self.mpc.set_param(
            n_horizon=self.n_horizon,
            t_step=self.t_step,
            state_discretization='discrete',
            store_full_solution=True
        )

        lterm = ca.mtimes([self.x.T, self.Q, self.x]) + ca.mtimes([self.u.T, self.R, self.u])
        mterm = ca.mtimes([self.x.T, self.Q, self.x])

        # x = self.mpc.model.x
        # u = self.mpc.model.u
        # y = ca.mtimes(ca.DM(self.C), x) + ca.mtimes(ca.DM(self.D), u)
        #
        # lterm = ca.mtimes([y.T, self.Q, y]) + ca.mtimes([self.u.T, self.R, self.u])
        # mterm = ca.mtimes([y.T, self.Q, y])

        self.mpc.set_objective(lterm=lterm, mterm=mterm)
        self.mpc.set_rterm(u=self.R[0,0])

        self.mpc.bounds['lower', '_u', 'u'] = -255.0
        self.mpc.bounds['upper', '_u', 'u'] = 255.0

        self.mpc.setup()

    def _setup_simulator(self):
        self.sim = do_mpc.simulator.Simulator(self.model)
        self.sim.set_param(t_step=self.t_step)
        self.sim.setup()

    def init_controller(self, x0):
        self.mpc.x0 = x0
        self.mpc.u0 = np.zeros((self.nu, 1))
        self.mpc.set_initial_guess()
        self.sim.x0 = x0

    def step(self, x_current):
        return self.mpc.make_step(x_current)
