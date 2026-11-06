# G-FOLD: Fuel-Optimal Powered Landing Guidance

[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![CVXPY](https://img.shields.io/badge/CVXPY-1.5+-green.svg)](https://www.cvxpy.org/)

**A powered-descent trajectory optimization example using lossless convexification (LCVX).**

![G-FOLD powered descent illustration](assets/hero.svg)

[Project homepage](https://paoshou-china.github.io/G-FOLD/) · [Source code](https://github.com/PaoShou-China/G-FOLD)

## What it contains

This repository provides Earth and Mars mission parameter presets, a two-stage trajectory solver, a Tkinter graphical interface, and Plotly trajectory and telemetry views.

The solver uses transformed control acceleration, log mass, and a thrust slack variable. It implements P3 and P4 optimization stages; consult the cited LCVX reference and the implementation for the exact assumptions and constraints. The supplied mission models are simplified powered-descent scenarios, not high-fidelity vehicle models.

## Installation

Python 3.10 or newer is recommended.

```bash
git clone https://github.com/PaoShou-China/G-FOLD.git
cd G-FOLD
python -m venv .venv
# Linux/macOS
source .venv/bin/activate
# Windows PowerShell: .venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
```

The GUI uses Tkinter, which is included with many Python installations. On some Linux distributions it must be installed separately through the system package manager.

## Quick start

Run the configured Earth mission from the command line:

```bash
python main.py
```

The current defaults in `main.py` select the Earth preset, use 50 discretization intervals, write `trajectory_results.json`, and display Plotly visualizations. To change the mission, interval count, output filename, or visualization setting, edit the corresponding constants near the top of `main.py`.

Launch the optional Tkinter interface with:

```bash
python gui.py
```

## Repository map

| File | Purpose |
| --- | --- |
| `main.py` | Runs the configured mission, saves trajectory results as JSON, and optionally displays plots. |
| `mission.py` | Defines shared mission parameters and Earth/Mars presets. |
| `solver.py` | Builds and solves the LCVX optimization stages. |
| `visualize.py` | Creates Plotly 3D trajectory and telemetry views. |
| `gui.py` | Tkinter interface for configuring and viewing a trajectory optimization. |

## Reproduction notes

- The mission values, discretization count, output path, and plot toggle are configured in `main.py`.
- Edit the mission classes in `mission.py` to change physical parameters.
- The result JSON includes mission metadata and state/control trajectories.
- Results depend on the chosen CVXPY solver backend and installed solver packages; check the solver status reported by the code.

## Model assumptions and limits

The repository represents the state as position, velocity, and log mass, and applies discretized powered-descent dynamics with glide-slope, thrust-pointing, throttle, velocity, and terminal constraints. It does not include a full six-degree-of-freedom vehicle model, atmosphere, terrain, or closed-loop flight controller.

## Reference

A. Açıkmeşe and S. R. Ploen, “Convex Programming Approach to Powered Descent Guidance,” *Journal of Guidance, Control, and Dynamics*, 30(5), 2007.

## License

See [LICENSE](LICENSE).
