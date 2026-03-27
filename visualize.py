"""Visualization module for rocket landing trajectory."""
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np


def plot_trajectory(tf: float, x: np.ndarray, u: np.ndarray, m: np.ndarray,
                    s: np.ndarray, zeta: np.ndarray, mission_config) -> None:
    """Plot 3D trajectory and time-series data.

    Args:
        tf: Flight time [s]
        x: State trajectory [6xN] - [r_z, r_x, r_y, v_z, v_x, v_y]
        u: Control acceleration [3xN] - [u_z, u_x, u_y]
        m: Mass trajectory [1xN]
        s: Sigma slack variable [1xN]
        zeta: Log-mass trajectory [1xN]
        mission_config: Mission configuration object
    """
    # Extract and trim data (exclude first and last points)
    t = np.linspace(0, tf, num=m.shape[1])[1:-1]
    r = np.array(x[0:3, 1:-1])  # Position
    v = np.array(x[3:6, 1:-1])  # Velocity
    zeta_arr = np.array(zeta).flatten()[1:-1]
    s_arr = np.array(s).flatten()[1:-1]
    u_arr = np.array(u)[:, 1:-1]
    m_arr = np.array(m).flatten()[1:-1]

    # Compute derived quantities
    thrust_mag = np.linalg.norm(u_arr, axis=0) * m_arr
    v_norm = np.linalg.norm(v, axis=0)
    u_angle = np.degrees(np.arccos(np.clip(u_arr[0] / np.linalg.norm(u_arr, axis=0), -1, 1)))

    # Create 3D trajectory plot
    traj_fig = _create_3d_trajectory(r, v, u_arr, m_arr, t, mission_config)
    traj_fig.show()

    # Create telemetry plots
    telem_fig = _create_telemetry_plots(t, r, v_norm, m_arr, thrust_mag, u_angle, s_arr, mission_config)
    telem_fig.show()


def _create_3d_trajectory(r: np.ndarray, v: np.ndarray, u: np.ndarray, 
                          m: np.ndarray, t: np.ndarray, cfg) -> go.Figure:
    """Create 3D trajectory visualization with animations and enhanced visuals."""
    fig = go.Figure()

    # Color scale based on altitude
    altitude = r[0]
    colorscale = [[0, '#3b82f6'], [0.5, '#8b5cf6'], [1, '#ef4444']]

    # Flight path with color gradient based on altitude
    for i in range(len(altitude) - 1):
        color_intensity = (altitude[i] - altitude.min()) / (altitude.max() - altitude.min() + 1e-6)
        fig.add_trace(go.Scatter3d(
            x=r[1, i:i+2], y=r[2, i:i+2], z=r[0, i:i+2],
            mode='lines',
            line=dict(
                color=color_intensity,
                colorscale=colorscale,
                width=6
            ),
            showlegend=False,
            hoverinfo='skip'
        ))

    # Add trajectory as a single line for legend
    fig.add_trace(go.Scatter3d(
        x=r[1], y=r[2], z=r[0],
        mode='lines',
        name='Flight Path',
        line=dict(color='rgba(100, 100, 100, 0.3)', width=1),
        showlegend=True
    ))

    # Ground plane
    _add_ground_plane(fig, r, cfg)

    # Glide slope cone
    _add_glide_slope_cone(fig, r, cfg)

    # Thrust vectors (sampled)
    _add_thrust_vectors(fig, r, u, cfg)

    # Velocity vectors (sampled)
    _add_velocity_vectors(fig, r, v, cfg)

    # Key points with enhanced markers
    _add_key_points(fig, cfg, r)

    # Animated rocket
    _add_animated_rocket(fig, r, t)

    # Layout with dark theme and better camera
    fig.update_layout(
        template='plotly_dark',
        scene=dict(
            aspectmode='data',
            xaxis_title='X Position (m)',
            yaxis_title='Y Position (m)',
            zaxis_title='Altitude (m)',
            camera=dict(
                eye=dict(x=2.0, y=2.0, z=1.5),
                center=dict(x=0, y=0, z=0),
                up=dict(x=0, y=0, z=1)
            ),
            bgcolor='#0f172a',
            xaxis=dict(gridcolor='#334155', showbackground=True, backgroundcolor='#1e293b'),
            yaxis=dict(gridcolor='#334155', showbackground=True, backgroundcolor='#1e293b'),
            zaxis=dict(gridcolor='#334155', showbackground=True, backgroundcolor='#1e293b')
        ),
        title=dict(
            text='🚀 Rocket Landing Trajectory (3D)',
            font=dict(size=20, color='white'),
            x=0.5
        ),
        legend=dict(
            y=0.98, x=0.02,
            yanchor='top',
            bgcolor='rgba(30, 41, 59, 0.9)',
            font=dict(color='white', size=14),
            itemsizing='constant'
        ),
        # Global marker size for legend
        uniformtext=dict(minsize=14, mode='hide'),
        paper_bgcolor='#0f172a',
        plot_bgcolor='#0f172a',
        updatemenus=[{
            'type': 'buttons',
            'showactive': False,
            'buttons': [{
                'label': '▶ Play Animation',
                'method': 'animate',
                'args': [None, {
                    'frame': {'duration': 50, 'redraw': True},
                    'fromcurrent': True,
                    'transition': {'duration': 0}
                }]
            }, {
                'label': '⏸ Pause',
                'method': 'animate',
                'args': [[None], {
                    'frame': {'duration': 0, 'redraw': False},
                    'mode': 'immediate',
                    'transition': {'duration': 0}
                }]
            }],
            'x': 0.1,
            'y': 0.05
        }],
        sliders=[{
            'steps': [
                {
                    'args': [[f'frame{i}'], {
                        'frame': {'duration': 0, 'redraw': True},
                        'mode': 'immediate',
                        'transition': {'duration': 0}
                    }],
                    'label': f'{t[i]:.1f}s',
                    'method': 'animate'
                }
                for i in range(0, len(t), max(1, len(t) // 20))
            ],
            'transition': {'duration': 0},
            'x': 0.1,
            'y': 0,
            'currentvalue': {'visible': True, 'prefix': 'Time: '},
            'len': 0.8
        }]
    )
    return fig


def _add_ground_plane(fig: go.Figure, r: np.ndarray, cfg) -> None:
    """Add ground plane at landing altitude."""
    r_max = max(np.max(np.abs(r[1])), np.max(np.abs(r[2]))) * 1.2
    if r_max < 1e-6:
        r_max = 100

    # Create grid
    x = np.linspace(-r_max, r_max, 20)
    y = np.linspace(-r_max, r_max, 20)
    X, Y = np.meshgrid(x, y)
    Z = np.full_like(X, cfg.landing_point[0])

    fig.add_trace(go.Surface(
        x=X, y=Y, z=Z,
        colorscale=[[0, '#1e293b'], [1, '#1e293b']],
        opacity=0.3,
        showscale=False,
        name='Ground',
        hoverinfo='skip'
    ))


def _add_glide_slope_cone(fig: go.Figure, r: np.ndarray, cfg) -> None:
    """Add glide slope constraint cone to the 3D plot."""
    r_max = max(np.max(np.abs(r[1])), np.max(np.abs(r[2]))) * 2.0  # Make cone even larger
    if r_max < 1e-6:
        return

    r_ = np.linspace(0, r_max, 30)
    a_ = np.linspace(0, 2 * np.pi, 60)
    R, P = np.meshgrid(r_, a_)
    X, Y = R * np.cos(P), R * np.sin(P)

    xf, yf, zf = r[1, -1], r[2, -1], r[0, -1]
    Z = R * np.tan(cfg.angle_gs) + zf

    fig.add_trace(go.Surface(
        x=X + xf, y=Y + yf, z=Z,
        colorscale='YlGnBu', opacity=0.25, showscale=False, name='Glide Slope'
    ))


def _add_thrust_vectors(fig: go.Figure, r: np.ndarray, u: np.ndarray, cfg) -> None:
    """Add thrust direction vectors to the 3D plot."""
    r_max = max(np.max(np.abs(r[1])), np.max(np.abs(r[2])))
    step = max(1, r.shape[1] // 10)

    for i in range(0, r.shape[1], step):
        u_mag = np.linalg.norm(u[:, i])
        if u_mag > 0.1:
            u_dir = u[:, i] / u_mag
            scale = (u_mag / cfg.thrust_max) * r_max * 0.2
            fig.add_trace(go.Scatter3d(
                x=[r[1, i], r[1, i] + scale * u_dir[1]],
                y=[r[2, i], r[2, i] + scale * u_dir[2]],
                z=[r[0, i], r[0, i] + scale * u_dir[0]],
                mode='lines',
                line=dict(color='rgba(239, 68, 68, 0.7)', width=3),
                showlegend=False,
                hoverinfo='skip'
            ))


def _add_velocity_vectors(fig: go.Figure, r: np.ndarray, v: np.ndarray, cfg) -> None:
    """Add velocity direction vectors to the 3D plot."""
    r_max = max(np.max(np.abs(r[1])), np.max(np.abs(r[2])))
    step = max(1, r.shape[1] // 10)
    v_max = np.max(np.linalg.norm(v, axis=0)) + 1e-6

    for i in range(0, r.shape[1], step):
        v_mag = np.linalg.norm(v[:, i])
        if v_mag > 0.1:
            v_dir = v[:, i] / v_mag
            scale = (v_mag / v_max) * r_max * 0.15
            fig.add_trace(go.Scatter3d(
                x=[r[1, i], r[1, i] + scale * v_dir[1]],
                y=[r[2, i], r[2, i] + scale * v_dir[2]],
                z=[r[0, i], r[0, i] + scale * v_dir[0]],
                mode='lines',
                line=dict(color='rgba(59, 130, 246, 0.7)', width=3),
                showlegend=False,
                hoverinfo='skip'
            ))


def _add_key_points(fig: go.Figure, cfg, r: np.ndarray) -> None:
    """Add start, target, and actual landing points to the 3D plot."""
    z0, x0, y0 = cfg.initial_state[0:3]
    zp, xp, yp = cfg.landing_point[0:3]

    points = [
        ([x0], [y0], [z0], '#22c55e', 'diamond', 'Start', 8),
        ([xp], [yp], [zp], '#ef4444', 'x', 'Target', 8),
        ([r[1, -1]], [r[2, -1]], [r[0, -1]], '#3b82f6', 'circle', 'Landing', 6)
    ]

    for x, y, z, color, symbol, name, size in points:
        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='markers',
            marker=dict(
                size=size,
                color=color,
                symbol=symbol,
                line=dict(color='white', width=1)
            ),
            name=name
        ))


def _add_animated_rocket(fig: go.Figure, r: np.ndarray, t: np.ndarray) -> None:
    """Add animated rocket marker with perspective scaling."""
    frames = []
    n_frames = min(len(t), 50)
    indices = np.linspace(0, len(t) - 1, n_frames, dtype=int)
    
    # Calculate distance from camera for perspective scaling
    # Camera is at approximately (2, 2, 1.5) in normalized coordinates
    # Use altitude as proxy for distance (higher = farther = smaller)
    altitudes = r[0]
    max_alt = altitudes.max()
    min_alt = altitudes.min()
    
    for idx in indices:
        # Scale size based on altitude (perspective effect)
        # Closer (lower altitude) = larger, Farther (higher altitude) = smaller
        alt_ratio = (altitudes[idx] - min_alt) / (max_alt - min_alt + 1e-6)
        size = 2 + 1 * (1 - alt_ratio)  # Size between 2 and 3
        
        frames.append(go.Frame(
            data=[go.Scatter3d(
                x=[r[1, idx]],
                y=[r[2, idx]],
                z=[r[0, idx]],
                mode='markers',
                marker=dict(
                    size=size,
                    color='#fbbf24',
                    symbol='diamond',
                    line=dict(color='white', width=1)
                ),
                name='Rocket'
            )],
            name=f'frame{idx}'
        ))

    fig.frames = frames

    # Add initial rocket position
    initial_size = 2.5  # Start with medium size
    fig.add_trace(go.Scatter3d(
        x=[r[1, 0]], y=[r[2, 0]], z=[r[0, 0]],
        mode='markers',
        marker=dict(
            size=initial_size,
            color='#fbbf24',
            symbol='diamond',
            line=dict(color='white', width=1)
        ),
        name='Rocket'
    ))


def _create_telemetry_plots(t: np.ndarray, r: np.ndarray, v_norm: np.ndarray,
                            m: np.ndarray, thrust: np.ndarray, u_angle: np.ndarray,
                            s: np.ndarray, cfg) -> go.Figure:
    """Create 2x3 grid of telemetry plots."""
    fig = make_subplots(
        rows=2, cols=3,
        subplot_titles=(
            "Velocity Magnitude", "Altitude", "Mass",
            "Thrust Magnitude", "Thrust Angle", "Sigma (Thrust Slack)"
        ),
        vertical_spacing=0.15, horizontal_spacing=0.1
    )

    # Row 1: Velocity, Altitude, Mass
    _add_scatter_with_limit(fig, t, v_norm, cfg.max_velocity, 'Velocity ||v||', '#3b82f6', 1, 1)
    _add_scatter(fig, t, r[0], 'Altitude r_z', '#22c55e', 1, 2)
    _add_scatter(fig, t, m, 'Mass m', '#8b5cf6', 1, 3)

    # Row 2: Thrust, Angle, Sigma
    _add_thrust_plot(fig, t, thrust, cfg, 2, 1)
    _add_angle_plot(fig, t, u_angle, cfg, 2, 2)
    _add_scatter(fig, t, s, 'Sigma σ', '#ec4899', 2, 3)

    # Update axes
    _update_axes(fig)

    fig.update_layout(
        title='Telemetry Data Over Time',
        showlegend=True,
        legend=dict(orientation='h', yanchor='bottom', y=-0.15, xanchor='center', x=0.5),
        height=700, width=1200,
        margin=dict(l=60, r=60, t=80, b=100),
        template='plotly_dark',
        paper_bgcolor='#0f172a',
        plot_bgcolor='#0f172a'
    )
    return fig


def _add_scatter(fig: go.Figure, x: np.ndarray, y: np.ndarray, name: str,
                 color: str, row: int, col: int) -> None:
    """Add a simple scatter trace to the figure."""
    fig.add_trace(go.Scatter(x=x, y=y, mode='lines', name=name, line=dict(color=color, width=2)), row=row, col=col)


def _add_scatter_with_limit(fig: go.Figure, x: np.ndarray, y: np.ndarray, limit: float,
                            name: str, color: str, row: int, col: int) -> None:
    """Add a scatter trace with a limit line."""
    _add_scatter(fig, x, y, name, color, row, col)
    fig.add_trace(go.Scatter(
        x=x, y=np.full(len(x), limit), mode='lines',
        name='Limit', line=dict(color='red', dash='dash', width=2), showlegend=False
    ), row=row, col=col)


def _add_thrust_plot(fig: go.Figure, t: np.ndarray, thrust: np.ndarray, cfg, row: int, col: int) -> None:
    """Add thrust magnitude plot with min/max bounds."""
    _add_scatter(fig, t, thrust, 'Thrust ||T||', '#f97316', row, col)
    fig.add_trace(go.Scatter(
        x=t, y=np.full(len(t), cfg.thrust_max), mode='lines',
        name='Max Thrust', line=dict(color='red', dash='dash', width=2), showlegend=False
    ), row=row, col=col)
    fig.add_trace(go.Scatter(
        x=t, y=np.full(len(t), cfg.thrust_lower_bound), mode='lines',
        name='Min Thrust', line=dict(color='red', dash='dot', width=2), showlegend=False
    ), row=row, col=col)


def _add_angle_plot(fig: go.Figure, t: np.ndarray, angle: np.ndarray, cfg, row: int, col: int) -> None:
    """Add thrust angle plot with limit."""
    _add_scatter(fig, t, angle, 'Thrust Angle θ', '#06b6d4', row, col)
    fig.add_trace(go.Scatter(
        x=t, y=np.full(len(t), np.degrees(cfg.angle_pt)), mode='lines',
        name='Max Angle', line=dict(color='red', dash='dash', width=2), showlegend=False
    ), row=row, col=col)


def _update_axes(fig: go.Figure) -> None:
    """Update all axes labels."""
    labels = [
        ('Time (s)', 'Velocity (m/s)', 1, 1),
        ('Time (s)', 'Altitude (m)', 1, 2),
        ('Time (s)', 'Mass (kg)', 1, 3),
        ('Time (s)', 'Thrust (N)', 2, 1),
        ('Time (s)', 'Angle (deg)', 2, 2),
        ('Time (s)', 'Sigma (m/s²)', 2, 3),
    ]
    for xlabel, ylabel, row, col in labels:
        fig.update_xaxes(title_text=xlabel, row=row, col=col, gridcolor='#334155')
        fig.update_yaxes(title_text=ylabel, row=row, col=col, gridcolor='#334155')


def run(tf: float, x: np.ndarray, u: np.ndarray, m: np.ndarray,
        s: np.ndarray, zeta: np.ndarray, mission_config) -> None:
    """Entry point for visualization."""
    plot_trajectory(tf, x, u, m, s, zeta, mission_config)
