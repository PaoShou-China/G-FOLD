"""GUI for Rocket Landing Trajectory Optimization.

A modern, user-friendly graphical interface for the G-FOLD trajectory optimizer.
Uses tkinter with custom styling.
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import json
import numpy as np
from datetime import datetime

from mission import EarthMission, MarsMission, MissionConfig
from solver import TrajectorySolver
from visualize import plot_trajectory


class RocketLandingGUI:
    """GUI application for rocket landing trajectory optimization."""

    FONT_FAMILY = "Microsoft YaHei"
    FONT_SIZE_NORMAL = 11
    FONT_SIZE_HEADER = 13
    FONT_SIZE_TITLE = 20

    def __init__(self, root: tk.Tk):
        """Initialize the GUI application."""
        self.root = root
        self.root.title("G-FOLD: Rocket Landing Trajectory Optimizer")
        self.root.geometry("1400x1000")
        self.root.minsize(1400, 1000)

        # Set theme colors
        self.colors = {
            'bg': '#0f172a',
            'card': '#1e293b',
            'card_light': '#334155',
            'accent': '#3b82f6',
            'accent_hover': '#2563eb',
            'success': '#10b981',
            'warning': '#f59e0b',
            'error': '#ef4444',
            'text': '#f1f5f9',
            'text_secondary': '#94a3b8',
            'border': '#475569'
        }

        self.root.configure(bg=self.colors['bg'])
        self._create_layout()

        # Initialize with Earth mission defaults
        self._load_mission_defaults('earth')

    def _create_layout(self) -> None:
        """Create the main GUI layout."""
        # Main container
        main_frame = tk.Frame(self.root, bg=self.colors['bg'])
        main_frame.pack(fill='both', expand=True, padx=20, pady=20)

        # Header
        self._create_header(main_frame)

        # Content area
        content = tk.Frame(main_frame, bg=self.colors['bg'])
        content.pack(fill='both', expand=True, pady=(20, 0))
        content.columnconfigure(0, weight=1)
        content.columnconfigure(1, weight=2)
        content.rowconfigure(0, weight=1)

        # Left panel: Configuration
        self._create_config_panel(content)

        # Right panel: Results
        self._create_results_panel(content)

    def _create_header(self, parent: tk.Frame) -> None:
        """Create the header."""
        header = tk.Frame(parent, bg=self.colors['bg'])
        header.pack(fill='x', pady=(0, 10))

        # Logo and title
        title_frame = tk.Frame(header, bg=self.colors['bg'])
        title_frame.pack(side='left')

        # ASCII Art Rocket
        rocket_frame = tk.Frame(title_frame, bg=self.colors['bg'])
        rocket_frame.pack(side='left', padx=(0, 10))
        
        rocket_lines = [
            "     _     ",
            "    | |    ",
            "    | |    ",
            "    | |    ",
            "   _\\|/_   ",
            "    \\|/    ",
            "     V     "
        ]
        for line in rocket_lines:
            tk.Label(
                rocket_frame,
                text=line,
                font=('Consolas', 5),
                bg=self.colors['bg'],
                fg=self.colors['accent']
            ).pack(anchor='center', pady=0)

        # Title and subtitle frame
        title_text_frame = tk.Frame(title_frame, bg=self.colors['bg'])
        title_text_frame.pack(side='left')

        tk.Label(
            title_text_frame,
            text="G-FOLD",
            font=(self.FONT_FAMILY, self.FONT_SIZE_TITLE, 'bold'),
            bg=self.colors['bg'],
            fg=self.colors['text']
        ).pack(anchor='w')

        tk.Label(
            title_text_frame,
            text="Rocket Landing Trajectory Optimizer  |  by paoshou",
            font=(self.FONT_FAMILY, 11),
            bg=self.colors['bg'],
            fg=self.colors['text_secondary']
        ).pack(anchor='w')

        # Version badge with ASCII rocket
        version_frame = tk.Frame(header, bg=self.colors['card'])
        version_frame.pack(side='right', padx=10)

        tk.Label(
            version_frame,
            text="▲",
            font=('Consolas', 10),
            bg=self.colors['card'],
            fg=self.colors['accent']
        ).pack(side='left', padx=(5, 0))

        tk.Label(
            version_frame,
            text="v0.1",
            font=(self.FONT_FAMILY, 10),
            bg=self.colors['card'],
            fg=self.colors['text_secondary'],
            padx=5,
            pady=3
        ).pack(side='left')

    def _create_config_panel(self, parent: tk.Frame) -> None:
        """Create the left configuration panel."""
        # Card container
        card = tk.Frame(parent, bg=self.colors['card'], bd=0)
        card.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        card.columnconfigure(1, weight=1)

        # Canvas with scrollbar
        canvas = tk.Canvas(card, bg=self.colors['card'], highlightthickness=0)
        scrollbar = tk.Scrollbar(card, orient='vertical', command=canvas.yview)
        scroll_frame = tk.Frame(canvas, bg=self.colors['card'])

        scroll_frame.bind(
            '<Configure>',
            lambda e: canvas.configure(scrollregion=canvas.bbox('all'))
        )

        canvas.create_window((0, 0), window=scroll_frame, anchor='nw', width=850)
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side='left', fill='both', expand=True, padx=15, pady=15)
        scrollbar.pack(side='right', fill='y')

        row = 0

        # Mission Profile
        self._create_section_header(scroll_frame, "Mission Profile", row)
        row += 1

        self.mission_type = ttk.Combobox(
            scroll_frame,
            values=['Earth Landing', 'Mars Landing'],
            state='readonly',
            width=30,
            font=(self.FONT_FAMILY, self.FONT_SIZE_NORMAL)
        )
        self.mission_type.set('Earth Landing')
        self.mission_type.grid(row=row, column=0, columnspan=2, sticky='w', pady=5)
        self.mission_type.bind('<<ComboboxSelected>>', self._on_mission_change)
        row += 1

        # Quick preset buttons
        presets = tk.Frame(scroll_frame, bg=self.colors['card'])
        presets.grid(row=row, column=0, columnspan=2, sticky='w', pady=(0, 15))

        tk.Button(
            presets,
            text="🌍 Earth",
            command=lambda: self._load_mission_defaults('earth'),
            bg=self.colors['card_light'],
            fg=self.colors['text'],
            font=(self.FONT_FAMILY, self.FONT_SIZE_NORMAL),
            relief='flat',
            padx=15,
            pady=5
        ).pack(side='left', padx=(0, 10))

        tk.Button(
            presets,
            text="🔴 Mars",
            command=lambda: self._load_mission_defaults('mars'),
            bg=self.colors['card_light'],
            fg=self.colors['text'],
            font=(self.FONT_FAMILY, self.FONT_SIZE_NORMAL),
            relief='flat',
            padx=15,
            pady=5
        ).pack(side='left')
        row += 1

        # Parameter sections - single column layout
        scroll_frame.columnconfigure(0, weight=0)
        scroll_frame.columnconfigure(1, weight=1)

        sections = [
            ("Environmental", [("Gravity (m/s²):", "gravity_var")]),
            ("Mass Parameters", [
                ("Dry Mass (kg):", "mass_dry_var"),
                ("Fuel Mass (kg):", "mass_fuel_var"),
            ]),
            ("Propulsion", [
                ("Max Thrust (N):", "thrust_max_var"),
                ("Min Throttle:", "throttle_min_var"),
                ("Max Throttle:", "throttle_max_var"),
                ("Specific Impulse (s):", "isp_var"),
            ]),
            ("Initial Position", [
                ("Altitude Z (m):", "pos_z_var"),
                ("Position X (m):", "pos_x_var"),
                ("Position Y (m):", "pos_y_var"),
            ]),
            ("Initial Velocity", [
                ("Velocity Z (m/s):", "vel_z_var"),
                ("Velocity X (m/s):", "vel_x_var"),
                ("Velocity Y (m/s):", "vel_y_var"),
            ]),
            ("Constraints", [
                ("Glide Slope (deg):", "angle_gs_var"),
                ("Thrust Angle (deg):", "angle_pt_var"),
                ("Max Velocity (m/s):", "max_vel_var"),
            ]),
            ("Solver Settings", [("Num Intervals:", "num_intervals_var")]),
        ]

        for title, fields in sections:
            self._create_section_header(scroll_frame, title, row)
            row += 1

            for label_text, var_name in fields:
                # Label
                tk.Label(
                    scroll_frame,
                    text=label_text,
                    bg=self.colors['card'],
                    fg=self.colors['text_secondary'],
                    font=(self.FONT_FAMILY, self.FONT_SIZE_NORMAL)
                ).grid(row=row, column=0, sticky='w', pady=1, padx=0)

                # Entry
                var = tk.Entry(
                    scroll_frame,
                    bg=self.colors['card_light'],
                    fg=self.colors['text'],
                    insertbackground=self.colors['text'],
                    relief='flat',
                    highlightbackground=self.colors['border'],
                    highlightthickness=1,
                    font=(self.FONT_FAMILY, self.FONT_SIZE_NORMAL),
                    width=15
                )
                var.grid(row=row, column=1, sticky='ew', pady=1, padx=(5, 0))
                setattr(self, var_name, var)
                row += 1

        self.num_intervals_var.insert(0, "50")

        # Run button
        self.run_button = tk.Button(
            scroll_frame,
            text="▶  RUN OPTIMIZATION",
            command=self._run_optimization,
            bg=self.colors['accent'],
            fg='white',
            font=(self.FONT_FAMILY, 14, 'bold'),
            relief='flat',
            padx=20,
            pady=5
        )
        self.run_button.grid(row=row, column=0, columnspan=2, sticky='w', pady=5)
        row += 1

        # Progress bar
        self.progress = ttk.Progressbar(
            scroll_frame,
            mode='indeterminate',
            length=400
        )
        self.progress.grid(row=row, column=0, columnspan=2, sticky='ew', pady=5)

    def _create_results_panel(self, parent: tk.Frame) -> None:
        """Create the right results panel."""
        # Card container
        card = tk.Frame(parent, bg=self.colors['card'], bd=0)
        card.grid(row=0, column=1, sticky='nsew', padx=(10, 0))
        card.columnconfigure(0, weight=1)
        card.rowconfigure(1, weight=1)

        # Status header
        status_frame = tk.Frame(card, bg=self.colors['card'])
        status_frame.pack(fill='x', padx=20, pady=(20, 10))

        self.status_indicator = tk.Canvas(
            status_frame,
            width=12,
            height=12,
            bg=self.colors['card'],
            highlightthickness=0
        )
        self.status_indicator.pack(side='left', padx=(0, 10))
        self.status_indicator.create_oval(2, 2, 10, 10, fill=self.colors['warning'])

        self.status_label = tk.Label(
            status_frame,
            text="Ready to run optimization",
            font=(self.FONT_FAMILY, self.FONT_SIZE_HEADER, 'bold'),
            bg=self.colors['card'],
            fg=self.colors['text']
        )
        self.status_label.pack(side='left')

        # Results text area - wider
        self.results_text = tk.Text(
            card,
            font=(self.FONT_FAMILY, 15),
            bg=self.colors['bg'],
            fg=self.colors['text'],
            insertbackground=self.colors['text'],
            relief='flat',
            highlightbackground=self.colors['border'],
            highlightthickness=1,
            wrap='word',
            width=70,
            height=25
        )
        self.results_text.pack(fill='both', expand=True, padx=20, pady=10)

        # Scrollbar
        scrollbar = tk.Scrollbar(self.results_text)
        scrollbar.pack(side='right', fill='y')
        self.results_text.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=self.results_text.yview)

        # Action buttons
        buttons = tk.Frame(card, bg=self.colors['card'])
        buttons.pack(fill='x', padx=20, pady=(10, 20))

        tk.Button(
            buttons,
            text="📊  Visualize",
            command=self._visualize,
            bg=self.colors['card_light'],
            fg=self.colors['text'],
            font=(self.FONT_FAMILY, self.FONT_SIZE_NORMAL),
            relief='flat',
            padx=15,
            pady=8
        ).pack(side='left', padx=(0, 10))

        tk.Button(
            buttons,
            text="💾  Save",
            command=self._save_results,
            bg=self.colors['card_light'],
            fg=self.colors['text'],
            font=(self.FONT_FAMILY, self.FONT_SIZE_NORMAL),
            relief='flat',
            padx=15,
            pady=8
        ).pack(side='left', padx=(0, 10))

        tk.Button(
            buttons,
            text="📋  Copy",
            command=self._copy_results,
            bg=self.colors['card_light'],
            fg=self.colors['text'],
            font=(self.FONT_FAMILY, self.FONT_SIZE_NORMAL),
            relief='flat',
            padx=15,
            pady=8
        ).pack(side='left')

    def _create_section_header(self, parent: tk.Frame, text: str, row: int) -> None:
        """Create a section header."""
        tk.Label(
            parent,
            text=text,
            font=(self.FONT_FAMILY, self.FONT_SIZE_HEADER, 'bold'),
            bg=self.colors['card'],
            fg=self.colors['accent']
        ).grid(row=row, column=0, columnspan=2, sticky='w', pady=(15, 5))

    def _on_mission_change(self, event=None) -> None:
        """Handle mission type change."""
        choice = self.mission_type.get()
        mission = 'earth' if 'Earth' in choice else 'mars'
        self._load_mission_defaults(mission)

    def _load_mission_defaults(self, mission_type: str) -> None:
        """Load default values for selected mission type."""
        if mission_type == 'earth':
            config = EarthMission()
        else:
            config = MarsMission()

        fields = [
            ("gravity_var", config.gravity_constant),
            ("mass_dry_var", config.mass_dry),
            ("mass_fuel_var", config.mass_fuel),
            ("thrust_max_var", config.thrust_max),
            ("throttle_min_var", config.throttle[0]),
            ("throttle_max_var", config.throttle[1]),
            ("isp_var", config.Isp),
            ("pos_z_var", config.initial_state[0]),
            ("pos_x_var", config.initial_state[1]),
            ("pos_y_var", config.initial_state[2]),
            ("vel_z_var", config.initial_state[3]),
            ("vel_x_var", config.initial_state[4]),
            ("vel_y_var", config.initial_state[5]),
            ("angle_gs_var", np.degrees(config.angle_gs)),
            ("angle_pt_var", np.degrees(config.angle_pt)),
            ("max_vel_var", config.max_velocity),
        ]

        for var_name, value in fields:
            widget = getattr(self, var_name)
            widget.delete(0, tk.END)
            widget.insert(0, str(value))

    def _get_mission_config(self) -> MissionConfig:
        """Create mission config from GUI values."""
        def get_float(var_name: str) -> float:
            widget = getattr(self, var_name)
            return float(widget.get())

        return MissionConfig(
            gravity=get_float("gravity_var"),
            mass_dry=get_float("mass_dry_var"),
            mass_fuel=get_float("mass_fuel_var"),
            thrust_max=get_float("thrust_max_var"),
            throttle=(get_float("throttle_min_var"), get_float("throttle_max_var")),
            Isp=get_float("isp_var"),
            initial_state=[
                get_float("pos_z_var"),
                get_float("pos_x_var"),
                get_float("pos_y_var"),
                get_float("vel_z_var"),
                get_float("vel_x_var"),
                get_float("vel_y_var")
            ],
            angle_gs=get_float("angle_gs_var"),
            angle_pt=get_float("angle_pt_var"),
            max_velocity=get_float("max_vel_var"),
            landing_point=[0.0, 0.0, 0.0]
        )

    def _run_optimization(self) -> None:
        """Run the optimization in a separate thread."""
        self.run_button.config(state='disabled')
        self.progress.start()
        self.status_label.config(text="Running optimization...", fg=self.colors['warning'])
        self.status_indicator.itemconfig(1, fill=self.colors['warning'])

        thread = threading.Thread(target=self._optimization_worker)
        thread.daemon = True
        thread.start()

    def _validate_mission_config(self, config) -> tuple[bool, str]:
        """Validate mission configuration for feasibility.
        
        Returns:
            Tuple of (is_valid, error_message)
        """
        errors = []
        
        # Check mass parameters
        if config.mass_dry <= 0:
            errors.append(f"Dry mass must be positive (got {config.mass_dry})")
        if config.mass_fuel <= 0:
            errors.append(f"Fuel mass must be positive (got {config.mass_fuel})")
        if config.mass_wet <= config.mass_dry:
            errors.append(f"Wet mass ({config.mass_wet}) must be greater than dry mass ({config.mass_dry})")
        
        # Check propulsion parameters
        if config.thrust_max <= 0:
            errors.append(f"Max thrust must be positive (got {config.thrust_max})")
        if config.thrust_lower_bound < 0:
            errors.append(f"Min thrust must be non-negative (got {config.thrust_lower_bound})")
        if config.thrust_upper_bound <= config.thrust_lower_bound:
            errors.append(f"Max thrust ({config.thrust_upper_bound}) must be greater than min thrust ({config.thrust_lower_bound})")
        if config.Isp <= 0:
            errors.append(f"Specific impulse must be positive (got {config.Isp})")
        
        # Check initial state
        initial_pos = config.initial_state[0:3]
        initial_vel = config.initial_state[3:6]
        if np.linalg.norm(initial_pos) < 1e-6:
            errors.append("Initial position is too close to origin")
        if np.linalg.norm(initial_vel) < 1e-6:
            errors.append("Initial velocity is nearly zero")
        
        # Check if landing is physically possible (simplified check)
        # Required delta-v to stop the vehicle
        delta_v_required = np.linalg.norm(initial_vel)
        # Available delta-v from fuel (Tsiolkovsky rocket equation approximation)
        if config.mass_dry > 0 and config.mass_wet > config.mass_dry:
            delta_v_available = config.Isp * config.gravity_constant * np.log(config.mass_wet / config.mass_dry)
            if delta_v_available < delta_v_required * 0.5:  # Need at least 50% margin
                errors.append(f"Insufficient fuel: available Δv ({delta_v_available:.1f} m/s) << required ({delta_v_required:.1f} m/s)")
        
        # Check constraint angles
        if config.angle_gs <= 0 or config.angle_gs >= np.pi/2:
            errors.append(f"Glide slope angle must be between 0 and 90 degrees")
        if config.angle_pt <= 0 or config.angle_pt >= np.pi:
            errors.append(f"Thrust pointing angle must be between 0 and 180 degrees")
        
        # Check gravity
        if config.gravity_constant <= 0:
            errors.append(f"Gravity must be positive (got {config.gravity_constant})")
        
        if errors:
            return False, "\n".join(errors)
        return True, ""

    def _optimization_worker(self) -> None:
        """Worker function for optimization."""
        try:
            config = self._get_mission_config()
            
            # Validate configuration before solving
            is_valid, error_msg = self._validate_mission_config(config)
            if not is_valid:
                self.root.after(0, self._on_validation_failed, error_msg)
                return
            
            num_intervals = int(self.num_intervals_var.get())
            solver = TrajectorySolver(config, N=num_intervals)
            result = solver.solve()
            self.root.after(0, self._on_optimization_complete, result, config)
        except Exception as e:
            self.root.after(0, self._on_optimization_error, str(e))

    def _on_validation_failed(self, error_msg: str) -> None:
        """Handle validation failure."""
        self.progress.stop()
        self.run_button.config(state='normal')
        self.status_label.config(text="Invalid parameters!", fg=self.colors['error'])
        self.status_indicator.itemconfig(1, fill=self.colors['error'])
        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(tk.END, "❌ Parameter validation failed:\n\n")
        self.results_text.insert(tk.END, error_msg)

    def _on_optimization_error(self, error_msg: str) -> None:
        """Handle optimization error."""
        self.progress.stop()
        self.run_button.config(state='normal')
        self.status_label.config(text="Optimization error!", fg=self.colors['error'])
        self.status_indicator.itemconfig(1, fill=self.colors['error'])
        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(tk.END, "❌ Optimization failed:\n\n")
        self.results_text.insert(tk.END, error_msg)

    def _on_optimization_complete(self, result, config) -> None:
        """Handle optimization completion."""
        self.progress.stop()
        self.run_button.config(state='normal')

        if result is None:
            self.status_label.config(text="Optimization failed!", fg=self.colors['error'])
            self.status_indicator.itemconfig(1, fill=self.colors['error'])
            self.results_text.insert(tk.END, "\n❌ Optimization failed!\n")
            return

        flight_time, x, u, m, s, zeta = result
        self.last_result = result
        self.last_config = config

        # Check if fuel consumed exceeds available fuel
        fuel_consumed = config.mass_wet - m[0, -1]
        available_fuel = config.mass_fuel
        
        if fuel_consumed > available_fuel * 1.001:  # 0.1% tolerance for numerical errors
            self.status_label.config(text="⚠ Fuel insufficient!", fg=self.colors['warning'])
            self.status_indicator.itemconfig(1, fill=self.colors['warning'])
            
            warning_str = f"""
{'='*50}
⚠ WARNING: INSUFFICIENT FUEL
{'='*50}

Available Fuel: {available_fuel:.2f} kg
Fuel Consumed:  {fuel_consumed:.2f} kg
Deficit:        {fuel_consumed - available_fuel:.2f} kg

The optimization result requires more fuel than available.
Please increase fuel mass or reduce mission requirements.

{'='*50}
"""
            self.results_text.delete(1.0, tk.END)
            self.results_text.insert(1.0, warning_str)
            return

        self.status_label.config(text="✓ Optimization completed successfully!", fg=self.colors['success'])
        self.status_indicator.itemconfig(1, fill=self.colors['success'])

        results_str = f"""
{'='*50}
OPTIMIZATION RESULTS
{'='*50}

Mission Type: {self.mission_type.get()}
Time of Flight: {flight_time:.2f} s
Initial Mass: {config.mass_wet:.2f} kg
Final Mass: {m[0, -1]:.2f} kg
Fuel Consumed: {config.mass_wet - m[0, -1]:.2f} kg
Fuel Remaining: {m[0, -1] - config.mass_dry:.2f} kg / {config.mass_fuel:.2f} kg ({((m[0, -1] - config.mass_dry) / config.mass_fuel * 100):.1f}%)

Initial State:
  Position: ({x[0, 0]:.1f}, {x[1, 0]:.1f}, {x[2, 0]:.1f}) m
  Velocity: ({x[3, 0]:.1f}, {x[4, 0]:.1f}, {x[5, 0]:.1f}) m/s

Final State:
  Position: ({x[0, -1]:.1f}, {x[1, -1]:.1f}, {x[2, -1]:.1f}) m
  Velocity: ({x[3, -1]:.1f}, {x[4, -1]:.1f}, {x[5, -1]:.1f}) m/s

{'='*50}
"""
        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(1.0, results_str)

        self._auto_save(flight_time, x, u, m, s, zeta, config)

    def _auto_save(self, flight_time, x, u, m, s, zeta, config) -> None:
        """Automatically save results."""
        results = {
            'timestamp': datetime.now().isoformat(),
            'mission_type': self.mission_type.get(),
            'intervals': int(self.num_intervals_var.get()),
            'flight_time': float(flight_time),
            'initial_mass': float(config.mass_wet),
            'final_mass': float(m[0, -1]),
            'fuel_consumed': float(config.mass_wet - m[0, -1]),
            'landing_point': [0.0, 0.0, 0.0],
            'trajectory': {
                'position': x[0:3, :].tolist(),
                'velocity': x[3:6, :].tolist(),
                'mass': m.tolist(),
                'thrust': u.tolist(),
                'sigma': s.tolist(),
                'zeta': zeta.tolist()
            }
        }

        filename = "trajectory_results.json"
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)

        self.results_text.insert(tk.END, f"\n✓ Results saved to: {filename}\n")

    def _visualize(self) -> None:
        """Open visualization."""
        if not hasattr(self, 'last_result') or self.last_result is None:
            messagebox.showwarning("No Data", "Please run optimization first!")
            return

        flight_time, x, u, m, s, zeta = self.last_result
        plot_trajectory(flight_time, x, u, m, s, zeta, self.last_config)

    def _save_results(self) -> None:
        """Save results to file."""
        if not hasattr(self, 'last_result') or self.last_result is None:
            messagebox.showwarning("No Data", "Please run optimization first!")
            return

        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
        )
        if filename:
            with open(filename, 'w') as f:
                json.dump({
                    'timestamp': datetime.now().isoformat(),
                    'mission_type': self.mission_type.get(),
                    'intervals': int(self.num_intervals_var.get()),
                    'flight_time': float(self.last_result[0]),
                    'initial_mass': float(self.last_config.mass_wet),
                    'final_mass': float(self.last_result[3][0, -1]),
                    'fuel_consumed': float(self.last_config.mass_wet - self.last_result[3][0, -1]),
                    'landing_point': [0.0, 0.0, 0.0],
                    'trajectory': {
                        'position': self.last_result[1][0:3, :].tolist(),
                        'velocity': self.last_result[1][3:6, :].tolist(),
                        'mass': self.last_result[3].tolist(),
                        'thrust': self.last_result[2].tolist(),
                        'sigma': self.last_result[4].tolist(),
                        'zeta': self.last_result[5].tolist()
                    }
                }, f, indent=2)
            messagebox.showinfo("Success", f"Results saved to {filename}")

    def _copy_results(self) -> None:
        """Copy results to clipboard."""
        text = self.results_text.get(1.0, tk.END)
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        messagebox.showinfo("Copied", "Results copied to clipboard!")


def main():
    """Entry point."""
    root = tk.Tk()
    app = RocketLandingGUI(root)
    root.mainloop()


if __name__ == '__main__':
    main()
