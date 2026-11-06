"""Command-line entry point for the G-FOLD powered-descent example.

Run with `python main.py --help` to see supported run-time options. The
module-level defaults remain available for scripts that import this module.
"""

from __future__ import annotations

import argparse
import json
import sys
import os
import tempfile
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Sequence

import numpy as np

from mission import EarthMission, MarsMission


# Backward-compatible defaults for existing scripts that import these names.
MISSION_TYPE = "earth"
NUM_INTERVALS = 50
OUTPUT_FILE = "trajectory_results.json"
ENABLE_VISUALIZATION = True


@dataclass(frozen=True)
class RunConfig:
    """Validated inputs for one command-line or programmatic run."""

    mission_type: str = MISSION_TYPE
    num_intervals: int = NUM_INTERVALS
    output_file: str = OUTPUT_FILE
    enable_visualization: bool = ENABLE_VISUALIZATION

    def __post_init__(self) -> None:
        normalized_mission = self.mission_type.strip().lower()
        if normalized_mission not in {"earth", "mars"}:
            raise ValueError("mission_type must be 'earth' or 'mars'.")
        if isinstance(self.num_intervals, bool) or not isinstance(self.num_intervals, (int, np.integer)) or self.num_intervals < 2:
            raise ValueError("num_intervals must be an integer greater than or equal to 2.")
        if not self.output_file:
            raise ValueError("output_file must not be empty.")
        object.__setattr__(self, "mission_type", normalized_mission)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Solve the configured powered-descent trajectory optimization problem."
    )
    parser.add_argument("--mission", choices=("earth", "mars"), default=MISSION_TYPE,
                        help="mission preset (default: %(default)s)")
    parser.add_argument("--intervals", type=int, default=NUM_INTERVALS,
                        help="number of discretization intervals (default: %(default)s)")
    parser.add_argument("--output", default=OUTPUT_FILE,
                        help="JSON output path (default: %(default)s)")
    parser.add_argument("--no-visualization", action="store_true",
                        help="save results without opening Plotly figures")
    return parser


def _mission_from_name(name: str):
    """Return a fresh mission preset by its CLI name."""
    return EarthMission() if name == "earth" else MarsMission()


def print_header(title: str, width: int = 60) -> None:
    """Print a formatted section header."""
    print(f"\n{'=' * width}")
    print(f"  {title}")
    print("=" * width)


def print_subheader(title: str, width: int = 60) -> None:
    """Print a formatted subsection header."""
    print(f"\n  {title}")
    print("-" * width)


def print_config(mission_config, mission_type: str, num_intervals: int) -> None:
    """Print the selected mission and discretization settings."""
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
    output_file: str | Path,
    mission_config,
    num_intervals: int,
    flight_time: float,
    m: np.ndarray,
    x: np.ndarray,
    u: np.ndarray,
    s: np.ndarray,
    zeta: np.ndarray,
) -> None:
    """Validate and atomically write finite solver output as standard JSON."""
    output_path = Path(output_file).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    numeric_values = {
        "flight_time": flight_time,
        "mass": m,
        "state": x,
        "control": u,
        "thrust slack": s,
        "log mass": zeta,
        "landing point": mission_config.landing_point,
    }
    for name, value in numeric_values.items():
        if not np.all(np.isfinite(np.asarray(value, dtype=float))):
            raise ValueError(f"{name} contains NaN or infinite values.")
    results = {
        "timestamp": datetime.now().isoformat(),
        "mission_type": mission_config.__class__.__name__,
        "intervals": num_intervals,
        "flight_time": float(flight_time),
        "initial_mass": float(mission_config.mass_wet),
        "final_mass": float(m[0, -1]),
        "fuel_consumed": float(mission_config.mass_wet - m[0, -1]),
        "landing_point": mission_config.landing_point.tolist(),
        "trajectory": {
            "position": x[0:3, :].tolist(),
            "velocity": x[3:6, :].tolist(),
            "mass": m.tolist(),
            "thrust": u.tolist(),
            "sigma": s.tolist(),
            "zeta": zeta.tolist(),
        },
    }
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output_path.parent,
            prefix=f".{output_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as result_file:
            temporary_path = Path(result_file.name)
            json.dump(results, result_file, indent=2, allow_nan=False)
            result_file.write("\n")
        os.replace(temporary_path, output_path)
    except Exception:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise
    print(f"  Results saved to: {output_path}")


def run(config: RunConfig) -> int:
    """Run the selected mission and return a process-style status code."""
    print_header("ROCKET TRAJECTORY OPTIMIZATION SYSTEM")
    mission_config = _mission_from_name(config.mission_type)
    print_config(mission_config, config.mission_type, config.num_intervals)
    print_header("STARTING OPTIMIZATION PROCESS")

    try:
        from solver import TrajectorySolver

        result = TrajectorySolver(mission_config, N=config.num_intervals).solve()
        if result is None:
            print("\n  OPTIMIZATION FAILED: solver returned no solution.")
            return 1

        flight_time, x, u, m, s, zeta = result
        print_subheader("OPTIMIZATION RESULTS")
        print(f"  Time of Flight.......... {flight_time:.2f} s")
        print(f"  Final Mass.............. {m[0, -1]:.2f} kg")
        print(f"  Fuel Consumed........... {mission_config.mass_wet - m[0, -1]:.2f} kg")
        save_results(config.output_file, mission_config, config.num_intervals,
                     flight_time, m, x, u, s, zeta)

        if config.enable_visualization:
            from visualize import plot_trajectory

            print("\n  Generating visualizations...")
            plot_trajectory(flight_time, x, u, m, s, zeta, mission_config)

        print_header("TRAJECTORY OPTIMIZATION COMPLETED SUCCESSFULLY")
        return 0
    except Exception as exc:
        print(f"\n  Error during trajectory run ({type(exc).__name__}): {exc}")
        return 1


def main(argv: Sequence[str] | None = None) -> int:
    """Parse CLI options and run; no arguments preserves the original defaults."""
    args = _build_parser().parse_args(argv)
    try:
        config = RunConfig(
            mission_type=args.mission,
            num_intervals=args.intervals,
            output_file=args.output,
            enable_visualization=not args.no_visualization,
        )
    except ValueError as exc:
        _build_parser().error(str(exc))
    return run(config)


if __name__ == "__main__":
    sys.exit(main())
