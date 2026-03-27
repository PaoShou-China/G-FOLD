"""
Main entry point for rocket landing trajectory optimization.
G-FOLD: Fuel-Optimal Powered Landing Guidance via Lossless Convexification

================================================================================
USAGE GUIDE
================================================================================

1. CONFIGURATION (Modify the variables below):
   
   MISSION_TYPE: Select mission scenario
       - 'earth' : Earth landing mission (high gravity, high thrust)
       - 'mars'  : Mars landing mission (low gravity, low thrust)
   
   NUM_INTERVALS: Number of discretization time intervals
       - Default: 20
       - Increase for higher accuracy (slower computation)
       - Decrease for faster computation (lower accuracy)
   
   OUTPUT_FILE: Path to save trajectory results (JSON format)
       - Default: 'trajectory_results.json'
   
   ENABLE_VISUALIZATION: Whether to show 3D trajectory plots
       - True: Display interactive plots after optimization
       - False: Skip visualization

2. CUSTOMIZING MISSION PARAMETERS:
   
   To modify mission parameters (mass, thrust, initial state, etc.),
   edit the corresponding class in mission.py:
   
   - EarthMission class: Earth landing parameters
   - MarsMission class: Mars landing parameters
   
   Key parameters to customize:
   - mass_dry: Dry mass of the vehicle (kg)
   - mass_fuel: Initial fuel mass (kg)
   - thrust_max: Maximum thrust (N)
   - Isp: Specific impulse (s)
   - initial_state: [r_z, r_x, r_y, v_z, v_x, v_y] (m, m/s)
   - landing_point: Target landing position [r_z, r_x, r_y] (m)
   - angle_gs: Glide slope angle (deg)
   - angle_pt: Thrust pointing angle (deg)

3. RUNNING THE OPTIMIZATION:
   
   Simply execute this script:
       python main.py
   
   The program will:
   a) Load mission configuration
   b) Estimate optimal flight time using golden section search
   c) Solve two-stage optimization (P3: landing error, P4: minimum fuel)
   d) Save results to JSON file
   e) Display visualization (if enabled)

4. OUTPUT FORMAT:
   
   The JSON output file contains:
   - timestamp: Execution time
   - mission_type: Mission class name
   - intervals: Number of discretization intervals
   - flight_time: Optimal time of flight (s)
   - initial_mass: Starting mass (kg)
   - final_mass: Mass at landing (kg)
   - fuel_consumed: Total fuel used (kg)
   - landing_point: Target landing coordinates
   - trajectory: Full state history
       - position: r_z, r_x, r_y over time
       - velocity: v_z, v_x, v_y over time
       - mass: Mass over time
       - thrust: Control acceleration u over time
       - sigma: Thrust slack variable over time
       - zeta: Log-mass over time

5. DEPENDENCIES:
   
   Required packages:
   - numpy: Numerical computations
   - cvxpy: Convex optimization solver
   - plotly: Interactive visualization
   
   Install with:
       pip install numpy cvxpy plotly

"""

import sys
import json
from datetime import datetime
from mission import EarthMission, MarsMission
from solver import TrajectorySolver
from visualize import plot_trajectory
import numpy as np

# ================================================================================
# USER CONFIGURATION - Modify these parameters as needed
# ================================================================================

MISSION_TYPE = 'earth'      # 'earth' or 'mars'
NUM_INTERVALS = 50          # Number of discretization intervals (higher = more accurate)
OUTPUT_FILE = 'trajectory_results.json'
ENABLE_VISUALIZATION = True


def print_header(title: str, width: int = 60) -> None:
    """Print a formatted section header."""
    print(f"\n{'=' * width}")
    print(f"  {title}")
    print('=' * width)


def print_subheader(title: str, width: int = 60) -> None:
    """Print a formatted subsection header."""
    print(f"\n  {title}")
    print('-' * width)


def print_config(mission_config, mission_type: str, num_intervals: int) -> None:
    """Print mission configuration details."""
    print_subheader("MISSION CONFIGURATION")
    lp = mission_config.landing_point
    print(f"  Mission Type............ {mission_type.capitalize()} Landing")
    print(f"  Discretization.......... {num_intervals} intervals")
    print(f"  Initial Mass............ {mission_config.mass_wet:.2f} kg")
    print(f"  Dry Mass................ {mission_config.mass_dry:.2f} kg")
    print(f"  Fuel Mass............... {mission_config.mass_fuel:.2f} kg")
    print(f"  Max Thrust.............. {mission_config.thrust_max:.2f} N")
    print(f"  Isp..................... {mission_config.Isp:.2f} s")
    print(f"  Gravity................. {mission_config.gravity_constant:.2f} m/s²")
    print(f"  Landing Point........... ({lp[0]:.0f}, {lp[1]:.0f}, {lp[2]:.0f}) m")


def save_results(
    output_file: str,
    mission_config,
    num_intervals: int,
    flight_time: float,
    m: np.ndarray,
    x: np.ndarray,
    u: np.ndarray,
    s: np.ndarray,
    zeta: np.ndarray
) -> None:
    """Save optimization results to JSON file."""
    results = {
        'timestamp': datetime.now().isoformat(),
        'mission_type': mission_config.__class__.__name__,
        'intervals': num_intervals,
        'flight_time': float(flight_time),
        'initial_mass': float(mission_config.mass_wet),
        'final_mass': float(m[0, -1]),
        'fuel_consumed': float(mission_config.mass_wet - m[0, -1]),
        'landing_point': mission_config.landing_point.tolist(),
        'trajectory': {
            'position': x[0:3, :].tolist(),
            'velocity': x[3:6, :].tolist(),
            'mass': m.tolist(),
            'thrust': u.tolist(),
            'sigma': s.tolist(),
            'zeta': zeta.tolist()
        }
    }
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2, default=lambda x: x.tolist() if hasattr(x, 'tolist') else x)
    print(f"  Results saved to: {output_file}")


def main() -> int:
    """Main entry point for trajectory optimization."""
    print_header("ROCKET TRAJECTORY OPTIMIZATION SYSTEM")

    # Select mission configuration
    mission_config = EarthMission() if MISSION_TYPE == 'earth' else MarsMission()
    print_config(mission_config, MISSION_TYPE, NUM_INTERVALS)

    # Run optimization
    print_header("STARTING OPTIMIZATION PROCESS")

    try:
        solver = TrajectorySolver(mission_config, N=NUM_INTERVALS)
        result = solver.solve()

        if result is None:
            print("\n  OPTIMIZATION FAILED")
            return 1

        flight_time, x, u, m, s, zeta = result

        # Display results
        print_subheader("OPTIMIZATION RESULTS")
        print(f"  Time of Flight.......... {flight_time:.2f} s")
        print(f"  Final Mass.............. {m[0, -1]:.2f} kg")
        print(f"  Fuel Consumed........... {mission_config.mass_wet - m[0, -1]:.2f} kg")

        # Save results
        save_results(OUTPUT_FILE, mission_config, NUM_INTERVALS, flight_time, m, x, u, s, zeta)

        # Visualization
        if ENABLE_VISUALIZATION:
            print("\n  Generating visualizations...")
            plot_trajectory(flight_time, x, u, m, s, zeta, mission_config)

        print_header("TRAJECTORY OPTIMIZATION COMPLETED SUCCESSFULLY")
        return 0

    except Exception as e:
        print(f"\n  Error: {e}")
        return 1


if __name__ == '__main__':
    import numpy as np  # Import here to avoid circular import issues
    sys.exit(main())
