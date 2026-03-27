"""Solver module for rocket landing trajectory optimization using Lossless Convexification (LCVX).

Reference: Açikmeşe and Ploen, "Convex Programming Approach to Powered Descent Guidance",
           Journal of Guidance, Control, and Dynamics, Vol. 30, No. 5, 2007.

For detailed theoretical background, see README.md.
"""

import numpy as np
import cvxpy as cp


class LcvxOptimizer:
    """LCVX optimizer for powered landing guidance.

    Implements the convex optimization problem using change of variables:
    - u = T/m (control acceleration)
    - sigma = Gamma/m (scaled thrust slack)
    - zeta = ln(m) (log-mass)
    """

    def __init__(self, N: int, problem_type: str, packed_data: tuple):
        """Initialize LCVX optimizer.

        Args:
            N: Number of discretization intervals
            problem_type: 'p3' for minimum landing error, 'p4' for minimum fuel
            packed_data: Tuple containing (x0, z0_term_inv, z0_term_log, g, rf, sparse_params)
        """
        self.N = N
        self.problem_type = 3 if problem_type == 'p3' else 4
        self.x0, self.z0_term_inv, self.z0_term_log, self.g, self.rf, self.sparse_params = packed_data
        self._unpack_params()
        self.dt = self.tf_ / self.N
        self._setup_variables()
        self._build_constraints()

    def _unpack_params(self) -> None:
        """Unpack sparse parameters into individual attributes."""
        params = self.sparse_params.flatten()
        (self.alpha_dt, self.V_max, self.y_gs_cot, self.p_cs_cos,
         self.m_wet_log, self.r1, self.r2, self.tf_) = params[:8]

    def _setup_variables(self) -> None:
        """Initialize CVXPY optimization variables."""
        # State vector: [r_z, r_x, r_y, v_z, v_x, v_y] for each time step
        self.x = cp.Variable((6, self.N), name='state')
        # Control acceleration: [u_z, u_x, u_y] = T/m for each time step
        self.u = cp.Variable((3, self.N), name='control')
        # Log-mass: zeta = ln(m) for each time step
        self.zeta = cp.Variable((1, self.N), name='log_mass')
        # Scaled thrust slack: sigma = Gamma/m for each time step
        self.s = cp.Variable((1, self.N), name='sigma')

    def _build_constraints(self) -> None:
        """Build convex constraints for the optimization problem."""
        self.con = []
        self._add_boundary_constraints()
        self._add_dynamics_constraints()
        self._add_path_constraints()

    def _add_boundary_constraints(self) -> None:
        """Add initial and terminal boundary conditions."""
        # Initial conditions
        self.con += [
            self.x[0:3, 0] == self.x0[0:3, 0],
            self.x[3:6, 0] == self.x0[3:6, 0],
            self.zeta[0, 0] == self.m_wet_log
        ]

        # Terminal conditions
        self.con += [
            self.x[3:6, self.N-1] == 0,
            self.s[0, self.N-1] == 0,
            self.u[:, 0] == self.s[0, 0] * np.array([1, 0, 0]),
            self.u[:, self.N-1] == 0
        ]

        # Problem-specific terminal constraints
        if self.problem_type == 3:
            self.con += [self.x[0, self.N-1] == 0]  # Altitude must be zero
        else:
            self.con += [self.x[0:3, self.N-1] == self.rf]  # Position must match target

    def _add_dynamics_constraints(self) -> None:
        """Add dynamics constraints using trapezoidal discretization."""
        dt_half = self.dt * 0.5
        alpha_dt_half = self.alpha_dt * 0.5

        for n in range(self.N - 1):
            # Velocity dynamics
            self.con += [
                self.x[3:6, n+1] == self.x[3:6, n] + dt_half *
                (self.u[:, n] + self.u[:, n+1] + 2 * self.g[:, 0])
            ]
            # Position dynamics
            self.con += [
                self.x[0:3, n+1] == self.x[0:3, n] + dt_half *
                (self.x[3:6, n+1] + self.x[3:6, n])
            ]
            # Mass dynamics
            self.con += [
                self.zeta[0, n+1] == self.zeta[0, n] - alpha_dt_half *
                (self.s[0, n] + self.s[0, n+1])
            ]

    def _add_path_constraints(self) -> None:
        """Add path constraints for each time step."""
        for n in range(self.N - 1):
            # Glide slope constraint
            r_rel = self.x[0:3, n] - self.x[0:3, self.N-1]
            self.con += [cp.norm(r_rel[1:3]) <= self.y_gs_cot * r_rel[0]]

            # Velocity limit
            self.con += [cp.norm(self.x[3:6, n]) <= self.V_max]

            # Thrust magnitude and pointing constraints
            self.con += [
                cp.norm(self.u[:, n]) <= self.s[0, n],
                self.u[0, n] >= self.p_cs_cos * self.s[0, n]
            ]

            # Thrust bounds via Taylor expansion
            if n > 0:
                self._add_thrust_bounds(n)

    def _add_thrust_bounds(self, n: int) -> None:
        """Add thrust bounds constraints using Taylor expansion."""
        zeta0 = self.z0_term_log[0, n]
        dzeta = self.zeta[0, n] - zeta0
        mu_1 = self.r1 * self.z0_term_inv[0, n]
        mu_2 = self.r2 * self.z0_term_inv[0, n]

        # Lower bound: sigma >= mu1 * [1 - dzeta + dzeta^2/2]
        self.con += [self.s[0, n] >= mu_1 * (1 - dzeta + 0.5 * dzeta ** 2)]
        # Upper bound: sigma <= mu2 * [1 - dzeta]
        self.con += [self.s[0, n] <= mu_2 * (1 - dzeta)]

    def solve(self) -> tuple | None:
        """Solve the convex optimization problem.

        Returns:
            Tuple of (obj_opt, x, u, m, s, zeta) if successful, None otherwise
        """
        # Build objective
        if self.problem_type == 3:
            objective = cp.Minimize(cp.norm(self.x[0:3, self.N-1] - self.rf))
        else:
            objective = cp.Maximize(self.zeta[0, self.N-1])

        problem = cp.Problem(objective, self.con)
        obj_opt = problem.solve(solver='CLARABEL', verbose=True)

        if self.zeta.value is None:
            return None

        m = np.exp(self.zeta.value)
        return obj_opt, self.x.value, self.u.value, m, self.s.value, self.zeta.value


class TrajectorySolver:
    """Trajectory solver using two-stage optimization approach."""

    def __init__(self, mission_config, N: int = 20):
        """Initialize trajectory solver.

        Args:
            mission_config: Mission configuration object
            N: Number of discretization intervals (default: 20)
        """
        self.config = mission_config
        self.N = N
        self.golden_ratio = (np.sqrt(5) - 1) * 0.5
        self.tol = 10.0  # Convergence tolerance for golden section search

    def estimate_time(self, problem_type: str, rf: np.ndarray) -> float:
        """Estimate optimal flight time using golden section search.

        Args:
            problem_type: 'p3' or 'p4'
            rf: Target landing position

        Returns:
            Optimal flight time
        """
        # Compute bounds
        initial_speed = np.linalg.norm(self.config.initial_state[3:6])
        t_min = self.config.mass_dry * initial_speed / self.config.thrust_upper_bound
        # Avoid division by zero when thrust_lower_bound is 0
        min_thrust = max(self.config.thrust_lower_bound, 0.01 * self.config.thrust_upper_bound)
        t_max = self.config.mass_fuel / (self.config.alpha * min_thrust)

        # Golden section search
        while (t_max - t_min) ** 2 > self.tol:
            dt = (t_max - t_min) * self.golden_ratio
            t1, t2 = t_min + dt, t_max - dt

            c1 = self._compute_cost(t1, problem_type, rf)
            c2 = self._compute_cost(t2, problem_type, rf)

            if c1 > c2:
                t_max = t1
            else:
                t_min = t2

        return (t_max + t_min) * 0.5

    def _compute_cost(self, time: float, problem_type: str, rf: np.ndarray) -> float:
        """Compute cost for a given flight time.

        Args:
            time: Flight time to evaluate
            problem_type: 'p3' or 'p4'
            rf: Target landing position

        Returns:
            Cost value (lower is better)
        """
        bundle_data = self._prepare_data(rf, time)
        lcvx = LcvxOptimizer(self.N, problem_type, bundle_data)
        result = lcvx.solve()

        if result is None:
            return float('inf')

        _, x, _, _, _, zeta = result

        if problem_type == 'p3':
            return np.linalg.norm(x[0:3, self.N - 1] - rf)
        return -zeta[0, self.N - 1]

    def _prepare_data(self, final_position: np.ndarray, flight_time: float) -> tuple:
        """Prepare data bundle for LCVX optimizer.

        Args:
            final_position: Target landing position
            flight_time: Flight time

        Returns:
            Tuple of (x0, z0_term_inv, z0_term_log, g, rf, sparse_params)
        """
        cfg = self.config
        dt = flight_time / self.N
        alpha_dt = cfg.alpha * dt
        t_array = np.linspace(0, (self.N - 1) * dt, self.N)

        # Precompute mass trajectory terms
        # Ensure mass doesn't go below dry mass (with small margin)
        min_mass = cfg.mass_dry * 1.01  # 1% margin above dry mass
        z0_term = cfg.mass_wet - cfg.alpha * cfg.thrust_upper_bound * t_array
        z0_term = np.maximum(z0_term, min_mass)  # Clamp to minimum mass
        z0_term_inv = (1 / z0_term).reshape(1, self.N)
        z0_term_log = np.log(z0_term).reshape(1, self.N)

        x0 = cfg.initial_state.reshape(6, 1)
        g = cfg.gravity_vector.reshape(3, 1)

        sparse_params = np.array([
            alpha_dt, cfg.max_velocity, cfg.angle_gs_cot, cfg.angle_pt_cos,
            cfg.mass_wet_log, cfg.thrust_lower_bound, cfg.thrust_upper_bound, flight_time
        ]).reshape(8, 1)

        return x0, z0_term_inv, z0_term_log, g, final_position, sparse_params

    def solve(self) -> tuple | None:
        """Execute two-stage trajectory optimization.

        Returns:
            Tuple of (flight_time, x, u, m, s, zeta) if successful, None otherwise
        """
        # Stage 1: P3 (Minimum Landing Error)
        landing_point = self.config.landing_point
        tf_p3 = self.estimate_time('p3', landing_point)

        data_p3 = self._prepare_data(landing_point, tf_p3)
        result_p3 = LcvxOptimizer(self.N, 'p3', data_p3).solve()

        if result_p3 is None:
            print('Cannot solve problem p3.')
            return None

        _, x_p3, _, _, _, _ = result_p3
        closest_point = x_p3[0:3, self.N - 1]

        # Stage 2: P4 (Minimum Fuel)
        tf_p4 = self.estimate_time('p4', closest_point)

        data_p4 = self._prepare_data(closest_point, tf_p4)
        result_p4 = LcvxOptimizer(self.N, 'p4', data_p4).solve()

        if result_p4 is None:
            print('Cannot solve problem p4.')
            return None

        print(f'Time Of Flight: {tf_p4}')
        return (tf_p4, *result_p4[1:])
