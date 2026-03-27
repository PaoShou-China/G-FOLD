"""Mission configuration for rocket landing trajectory optimization."""
import numpy as np


class MissionConfig:
    """Base class for mission configuration."""

    def __init__(
        self,
        gravity: float,
        mass_dry: float,
        mass_fuel: float,
        thrust_max: float,
        throttle: tuple[float, float],
        Isp: float,
        initial_state: np.ndarray,
        angle_gs: float,
        angle_pt: float,
        max_velocity: float,
        landing_point: np.ndarray
    ):
        """Initialize mission configuration.

        Args:
            gravity: Gravity magnitude [m/s²]
            mass_dry: Dry mass [kg]
            mass_fuel: Fuel mass [kg]
            thrust_max: Maximum thrust [N]
            throttle: (min, max) throttle fractions
            Isp: Specific impulse [s]
            initial_state: [r_z, r_x, r_y, v_z, v_x, v_y] [m, m/s]
            angle_gs: Glide slope angle [deg]
            angle_pt: Thrust pointing angle [deg]
            max_velocity: Maximum velocity [m/s]
            landing_point: Target landing position [r_z, r_x, r_y] [m]
        """
        # Environmental parameters
        self.gravity_vector = np.array([-gravity, 0.0, 0.0])
        self.gravity_constant = gravity

        # Mass parameters
        self.mass_dry = mass_dry
        self.mass_fuel = mass_fuel
        self.mass_wet = mass_dry + mass_fuel

        # Propulsion parameters
        self.thrust_max = thrust_max
        self.throttle = throttle
        self.thrust_lower_bound = thrust_max * throttle[0]
        self.thrust_upper_bound = thrust_max * throttle[1]
        self.Isp = Isp
        self.alpha = 1 / (gravity * Isp)

        # Initial state
        self.initial_state = np.array(initial_state)

        # Constraint angles
        self.angle_gs = np.radians(angle_gs)
        self.angle_pt = np.radians(angle_pt)
        self.angle_gs_cot = 1 / np.tan(self.angle_gs)
        self.angle_pt_cos = np.cos(self.angle_pt)

        # Mass logarithms for convex optimization
        self.mass_dry_log = np.log(mass_dry)
        self.mass_wet_log = np.log(self.mass_wet)

        # Velocity constraint
        self.max_velocity = max_velocity

        # Target landing point
        self.landing_point = np.array(landing_point)


class EarthMission(MissionConfig):
    """Earth landing mission configuration."""

    def __init__(self):
        super().__init__(
            gravity=9.81,
            mass_dry=5000.0,
            mass_fuel=1200.0,
            thrust_max=172557.234375,
            throttle=(0.1, 1.0),
            Isp=156.83880615234375,
            initial_state=[2000.0, 100.0, 360.0, -100.0, 0.0, 0.0],
            angle_gs=30.0,
            angle_pt=45.0,
            max_velocity=500.0,
            landing_point=[0.0, 0.0, 0.0]
        )


class MarsMission(MissionConfig):
    """Mars landing mission configuration."""

    def __init__(self):
        super().__init__(
            gravity=3.71,
            mass_dry=2000.0,
            mass_fuel=800.0,
            thrust_max=24000.0,
            throttle=(0.2, 1.0),
            Isp=203.94,
            initial_state=[1000.0, 360.0, -560.0, -10.0, -5.0, -5.0],
            angle_gs=30.0,
            angle_pt=45.0,
            max_velocity=100.0,
            landing_point=[0.0, 0.0, 0.0]
        )
