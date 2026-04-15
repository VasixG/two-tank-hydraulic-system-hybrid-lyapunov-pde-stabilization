from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection

from core.types import Array
from experiments.two_tank_setup import TwoTankHybridSetup
from simulation.simulator import SimulationResult

PROJECT_ROOT = Path(__file__).resolve().parents[2]
FIGURE_DIR = PROJECT_ROOT / "figures"
ANIMATION_DIR = PROJECT_ROOT / "animations"

NOMINAL_LEVEL = 2.5
VOLUME_MARGIN = 1.0


def make_figure_dir(*parts: str) -> Path:
    path = FIGURE_DIR.joinpath(*parts)
    path.mkdir(parents=True, exist_ok=True)
    return path


def make_animation_dir(*parts: str) -> Path:
    path = ANIMATION_DIR.joinpath(*parts)
    path.mkdir(parents=True, exist_ok=True)
    return path


def valid_mask(xs: Array) -> Array:
    return np.isfinite(xs[:, 0]) & np.isfinite(xs[:, 1])


def state_norm(xs: Array) -> Array:
    return np.linalg.norm(xs, axis=1)


def finite_state_samples(*results: SimulationResult | None) -> Array:
    samples = [np.zeros((1, 2), dtype=float)]
    for result in results:
        if result is None:
            continue
        valid = valid_mask(result.x)
        if np.any(valid):
            samples.append(result.x[valid])
    return np.vstack(samples)


def target_volume_for_states(states: Array) -> float:
    min_deviation = float(np.min(states))
    return max(NOMINAL_LEVEL, -min_deviation + VOLUME_MARGIN)


def tank_volume_limits(states: Array, target_volume: float) -> tuple[float, float]:
    volumes = target_volume + states
    lower = max(0.0, float(np.min(volumes)) - VOLUME_MARGIN)
    upper = float(np.max(volumes)) + VOLUME_MARGIN
    return lower, max(upper, lower + VOLUME_MARGIN)


def absolute_volume(h: float, target_volume: float) -> float:
    if not np.isfinite(h):
        return target_volume
    return float(target_volume + h)


def mode_indicator(result: SimulationResult) -> Array:
    values = np.full(len(result.mode), np.nan, dtype=float)
    for idx, mode in enumerate(result.mode):
        if mode == "outer":
            values[idx] = 1.0
        elif mode == "local":
            values[idx] = 0.0
    return values


def phase_boundary_points(setup: TwoTankHybridSetup, samples: int = 400) -> tuple[Array, Array]:
    theta = np.linspace(0.0, 2.0 * np.pi, samples)
    circle = np.vstack([np.cos(theta), np.sin(theta)])
    eig_values, eig_vectors = np.linalg.eigh(setup.lyapunov.P)
    ellipse = eig_vectors @ (np.sqrt(setup.hybrid_controller.sigma_level / eig_values)[:, None] * circle)
    return ellipse[0], ellipse[1]


def theoretical_tau_in(setup: TwoTankHybridSetup, x0: Array) -> float | None:
    w0 = setup.lyapunov.value(x0)
    a = setup.hybrid_controller.sigma_level
    if w0 <= a:
        return None
    return float(np.log(w0 / a))


def print_summary(label: str, setup: TwoTankHybridSetup, x0: Array, result: SimulationResult) -> None:
    valid = valid_mask(result.x)
    last = np.where(valid)[0][-1]
    tau_in = theoretical_tau_in(setup, x0)

    print(f"Controller: {label}")
    print(f"  alpha={setup.system.params.alpha}, kappa={setup.system.params.kappa}, r={setup.system.params.r}")
    print(f"  sigma_level={setup.hybrid_controller.sigma_level}")
    print(f"  x0={x0}, W(x0)={setup.lyapunov.value(x0):.6f}")
    print(f"  last simulated time={result.t[last]:.4f}")
    print(f"  final state={result.x[last]}")
    print(f"  final W={result.W[last]:.6f}")
    print(f"  completed full horizon={bool(result.alive)}")
    if tau_in is not None:
        print(f"  theoretical entrance time: tau_in={tau_in:.4f}")


def add_time_colored_phase(
    ax,
    x: Array,
    y: Array,
    t: Array,
    xlabel: str,
    ylabel: str,
    title: str,
):
    if len(x) < 2:
        ax.scatter(x, y, s=70, color="tab:blue")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True)
        return None

    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    norm = plt.Normalize(t.min(), t.max())
    line_collection = LineCollection(segments, cmap="plasma", norm=norm, linewidth=2)
    line_collection.set_array(t[:-1])
    artist = ax.add_collection(line_collection)

    ax.scatter(x[0], y[0], marker="o", color="green", s=140, label="Start", zorder=5, alpha=0.85)
    ax.scatter(x[-1], y[-1], marker="x", color="red", s=140, label="Finish", zorder=5, alpha=0.85)

    x_range = max(float(np.max(x) - np.min(x)), 1e-3)
    y_range = max(float(np.max(y) - np.min(y)), 1e-3)
    ax.set_xlim(np.min(x) - 0.1 * x_range, np.max(x) + 0.1 * x_range)
    ax.set_ylim(np.min(y) - 0.1 * y_range, np.max(y) + 0.1 * y_range)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True)
    ax.legend()
    return artist


def save_hybrid_plots(
    setup: TwoTankHybridSetup,
    result: SimulationResult,
    x0: Array,
    output_dir: Path,
    figure_title: str,
) -> None:
    valid = valid_mask(result.x)
    t_valid = result.t[valid]
    xs_valid = result.x[valid]
    us_valid = result.u[valid]
    ws_valid = result.W[valid]
    mode_values = mode_indicator(result)
    tau_in = theoretical_tau_in(setup, x0)
    boundary_x, boundary_y = phase_boundary_points(setup)
    a = setup.hybrid_controller.sigma_level

    plt.figure(figsize=(15, 10), num=figure_title)

    plt.subplot(3, 2, 1)
    plt.plot(t_valid, xs_valid[:, 0], label="h1")
    plt.plot(t_valid, xs_valid[:, 1], label="h2")
    plt.xlim([result.t[0], result.t[-1]])
    plt.xlabel("Time (s)")
    plt.ylabel("Level deviation")
    plt.title("Tank Level Deviations")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 2, 2)
    plt.plot(t_valid, us_valid[:, 0], label="u1")
    plt.plot(t_valid, us_valid[:, 1], label="u2")
    plt.xlim([result.t[0], result.t[-1]])
    plt.xlabel("Time (s)")
    plt.ylabel("Control input")
    plt.title("Control Inputs")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 2, 3)
    phase_ax = plt.gca()
    line = add_time_colored_phase(
        phase_ax,
        xs_valid[:, 0],
        xs_valid[:, 1],
        t_valid,
        "h1",
        "h2",
        "Phase Portrait",
    )
    phase_ax.plot(boundary_x, boundary_y, "--", color="black", linewidth=1.5, label=r"$W=a$")
    phase_ax.scatter([0], [0], marker="+", color="black", s=80, label="Origin")
    phase_ax.legend()
    if line is not None:
        cb = plt.colorbar(line, ax=phase_ax)
        cb.set_label("Time (s)")

    plt.subplot(3, 2, 4)
    plt.plot(t_valid, ws_valid, label="W(x)")
    plt.axhline(a, color="red", linestyle="--", label="a")
    if tau_in is not None:
        plt.axvline(tau_in, color="black", linestyle=":", label=r"$\tau_{in}$")
        outer_mask = mode_values[valid] == 1.0
        if np.any(outer_mask):
            t_outer = t_valid[outer_mask]
            plt.plot(
                t_outer,
                setup.lyapunov.value(x0) * np.exp(-(t_outer - result.t[0])),
                linestyle="--",
                color="tab:green",
                label=r"$W(x_0)e^{-t}$",
            )
    plt.xlim([result.t[0], result.t[-1]])
    plt.xlabel("Time (s)")
    plt.ylabel("W")
    plt.title("Lyapunov Function")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(result.t, mode_values, label="Hybrid mode")
    if tau_in is not None:
        plt.axvline(tau_in, color="black", linestyle=":", label=r"$\tau_{in}$")
    plt.xlim([result.t[0], result.t[-1]])
    plt.ylim([-0.1, 1.1])
    plt.yticks([0, 1], ["local", "PDE outer"])
    plt.xlabel("Time (s)")
    plt.ylabel("Mode")
    plt.title("Hybrid Switching Logic")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(output_dir / "plots.png")
    plt.close()


def create_tank_animation(
    setup: TwoTankHybridSetup,
    result: SimulationResult,
    output_path: Path,
    title: str,
    mode_label: str,
) -> None:
    valid = valid_mask(result.x)
    boundary_x, boundary_y = phase_boundary_points(setup)
    mode_values = mode_indicator(result)
    state_samples = finite_state_samples(result)
    target_volume = target_volume_for_states(state_samples)
    min_volume, max_volume = tank_volume_limits(state_samples, target_volume)

    fig = plt.figure(figsize=(13, 6))
    gs = fig.add_gridspec(1, 2, width_ratios=[1, 1.2])

    ax_tanks = fig.add_subplot(gs[0, 0])
    ax_phase = fig.add_subplot(gs[0, 1])

    tank_width = 0.8
    tank_gap = 0.8
    tank_x = [0.0, tank_width + tank_gap]

    ax_tanks.set_xlim(-0.3, tank_x[1] + tank_width + 0.3)
    ax_tanks.set_ylim(min_volume, max_volume)
    ax_tanks.set_aspect("equal")
    ax_tanks.set_title(title)
    ax_tanks.set_xticks([])
    ax_tanks.set_ylabel("Tank volume (L)")

    for x_left in tank_x:
        ax_tanks.plot([x_left, x_left], [min_volume, max_volume], "k-", linewidth=2)
        ax_tanks.plot([x_left + tank_width, x_left + tank_width], [min_volume, max_volume], "k-", linewidth=2)
        ax_tanks.plot([x_left, x_left + tank_width], [min_volume, min_volume], "k-", linewidth=2)

    ax_tanks.axhline(
        target_volume,
        color="black",
        linestyle="--",
        linewidth=1,
        alpha=0.7,
    )

    water1 = plt.Rectangle(
        (tank_x[0], min_volume),
        tank_width,
        target_volume - min_volume,
        alpha=0.55,
        color="royalblue",
    )
    water2 = plt.Rectangle(
        (tank_x[1], min_volume),
        tank_width,
        target_volume - min_volume,
        alpha=0.55,
        color="crimson",
    )
    ax_tanks.add_patch(water1)
    ax_tanks.add_patch(water2)

    all_pts = [result.x[valid]]
    lim = max(2.5, float(np.max(np.abs(np.vstack(all_pts)))) + 0.5)

    ax_phase.set_xlim(-lim, lim)
    ax_phase.set_ylim(-lim, lim)
    ax_phase.set_aspect("equal")
    ax_phase.grid(True)
    ax_phase.set_title("Phase Portrait and Switching Boundary")
    ax_phase.set_xlabel("h1")
    ax_phase.set_ylabel("h2")
    ax_phase.plot(boundary_x, boundary_y, "--", linewidth=1.5, label=r"$W=a$")
    ax_phase.scatter([0], [0], marker="x", s=60, label="Origin")

    (line_main,) = ax_phase.plot([], [], linewidth=2, label=mode_label)
    (point_main,) = ax_phase.plot([], [], "o", markersize=8)

    ax_phase.legend(loc="upper right")

    def init():
        water1.set_y(min_volume)
        water2.set_y(min_volume)
        water1.set_height(target_volume - min_volume)
        water2.set_height(target_volume - min_volume)
        line_main.set_data([], [])
        point_main.set_data([], [])
        artists = [water1, water2, line_main, point_main]
        return tuple(artists)

    def update(frame: int):
        h1_value, h2_value = result.x[frame]
        volume1 = absolute_volume(h1_value, target_volume)
        volume2 = absolute_volume(h2_value, target_volume)
        water1.set_y(min_volume)
        water2.set_y(min_volume)
        water1.set_height(max(0.0, volume1 - min_volume))
        water2.set_height(max(0.0, volume2 - min_volume))

        if np.isfinite(result.x[frame, 0]) and np.isfinite(result.x[frame, 1]):
            line_main.set_data(result.x[: frame + 1, 0], result.x[: frame + 1, 1])
            point_main.set_data([result.x[frame, 0]], [result.x[frame, 1]])

        artists = [water1, water2, line_main, point_main]
        return tuple(artists)

    step = max(1, len(result.t) // 240)
    animation = FuncAnimation(
        fig,
        update,
        frames=range(0, len(result.t), step),
        interval=30,
        blit=True,
        init_func=init,
    )

    animation.save(output_path, writer="pillow", fps=24)
    plt.close(fig)
