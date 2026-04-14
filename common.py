from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection

ROOT_DIR = Path(__file__).parent
OUTPUT_DIR = ROOT_DIR / "gfx"

# =========================
# Parameters
# =========================
alpha = 2.4  # intensity of destabilizing nonlinear inflow effect
kappa = 0.8  # coupling between tanks
r = 1.0  # saturation scale in nonlinear term
k = 1.2  # local linear feedback gain
a = 0.8  # switching threshold: W = h1^2 + h2^2 <= a

X0 = np.array([2.2, 1.8], dtype=float)

# Simulation settings
dt = 0.01
T = 18.0
N = int(T / dt)

# Clipping threshold for runaway local trajectory
STATE_MAX = 8.0

# Visualization settings
NOMINAL_LEVEL = 2.5
TANK_HEIGHT = 6.0


# =========================
# Model
# =========================
def f(x: np.ndarray) -> np.ndarray:
    """Open-loop drift for the two-tank model."""
    h1, h2 = x
    nl1 = alpha * (h1**2 / (r**2 + h1**2)) * h1
    nl2 = alpha * (h2**2 / (r**2 + h2**2)) * h2
    c12 = kappa * (h2 - h1)
    c21 = kappa * (h1 - h2)
    return np.array([nl1 + c12, nl2 + c21], dtype=float)


def W(x: np.ndarray) -> float:
    """Quadratic Lyapunov function from the PDF."""
    return float(np.dot(x, x))


def u_loc(x: np.ndarray) -> np.ndarray:
    """Local linear stabilizer."""
    return -k * x


def u_ext(x: np.ndarray) -> np.ndarray:
    """
    External PDE-based control for G = I and W = ||x||^2.

    This is the explicit formula derived in converse_control.pdf:
        u_ext = - (W + gradW^T f) / ||gradW||^2 * gradW
    with gradW = 2x.
    """
    w = W(x)
    if w < 1e-12:
        return np.zeros_like(x)

    grad_w = 2.0 * x
    numerator = w + float(np.dot(grad_w, f(x)))
    denominator = float(np.dot(grad_w, grad_w))
    return -(numerator / denominator) * grad_w


def u_hybrid(x: np.ndarray) -> np.ndarray:
    """Hybrid law: outer PDE control, inner local control."""
    if W(x) > a:
        return u_ext(x)
    return u_loc(x)


def control_for_mode(x: np.ndarray, mode: str) -> np.ndarray:
    if mode == "local":
        return u_loc(x)
    if mode == "hybrid":
        return u_hybrid(x)
    raise ValueError("mode must be 'local' or 'hybrid'")


def rhs(x: np.ndarray, mode: str) -> np.ndarray:
    return f(x) + control_for_mode(x, mode)


# =========================
# Integrator
# =========================
def rk4_step(x: np.ndarray, dt_value: float, mode: str) -> np.ndarray:
    k1 = rhs(x, mode)
    k2 = rhs(x + 0.5 * dt_value * k1, mode)
    k3 = rhs(x + 0.5 * dt_value * k2, mode)
    k4 = rhs(x + dt_value * k3, mode)
    return x + (dt_value / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate(x0: np.ndarray, mode: str):
    xs = np.full((N + 1, 2), np.nan)
    us = np.full((N + 1, 2), np.nan)
    ws = np.full(N + 1, np.nan)
    region = np.zeros(N + 1, dtype=int)  # 0=inner(local), 1=outer(PDE)
    alive = np.ones(N + 1, dtype=bool)

    x = x0.astype(float).copy()
    xs[0] = x
    ws[0] = W(x)
    us[0] = control_for_mode(x, mode)
    region[0] = int(ws[0] > a) if mode == "hybrid" else 0

    for i in range(N):
        if not np.all(np.isfinite(x)) or np.max(np.abs(x)) > STATE_MAX:
            alive[i + 1 :] = False
            break

        x_new = rk4_step(x, dt, mode)

        if not np.all(np.isfinite(x_new)) or np.max(np.abs(x_new)) > STATE_MAX:
            alive[i + 1 :] = False
            break

        x = x_new
        xs[i + 1] = x
        ws[i + 1] = W(x)
        us[i + 1] = control_for_mode(x, mode)
        region[i + 1] = int(ws[i + 1] > a) if mode == "hybrid" else 0

    t = np.linspace(0.0, N * dt, N + 1)
    return t, xs, us, ws, region, alive


def simulate_all(x0: np.ndarray = X0):
    local = simulate(x0, "local")
    hybrid = simulate(x0, "hybrid")
    return local, hybrid


def theoretical_tau_in(x0: np.ndarray) -> Optional[float]:
    w0 = W(x0)
    if w0 <= a:
        return None
    return float(np.log(w0 / a))


# =========================
# Output helpers
# =========================
def make_output_dir(*parts: str) -> Path:
    path = OUTPUT_DIR.joinpath(*parts)
    path.mkdir(parents=True, exist_ok=True)
    return path


def valid_mask(xs: np.ndarray) -> np.ndarray:
    return np.isfinite(xs[:, 0]) & np.isfinite(xs[:, 1])


def phase_boundary_points(samples: int = 400):
    theta = np.linspace(0.0, 2.0 * np.pi, samples)
    radius = np.sqrt(a)
    return radius * np.cos(theta), radius * np.sin(theta)


def state_norm(xs: np.ndarray) -> np.ndarray:
    return np.linalg.norm(xs, axis=1)


def clamp_level(h: float) -> float:
    return float(np.clip(NOMINAL_LEVEL + h, 0.0, TANK_HEIGHT))


def print_summary(mode: str, x0: np.ndarray, t: np.ndarray, xs: np.ndarray, ws: np.ndarray, alive: np.ndarray):
    valid = valid_mask(xs)
    last = np.where(valid)[0][-1]
    print(f"Mode: {mode}")
    print(f"  alpha={alpha}, kappa={kappa}, r={r}, k={k}, a={a}")
    print(f"  x0={x0}, W(x0)={W(x0):.4f}")
    print(f"  last simulated time={t[last]:.4f}")
    print(f"  final state={xs[last]}")
    print(f"  final W={ws[last]:.6f}")
    print(f"  completed full horizon={bool(alive[-1])}")
    if mode == "hybrid":
        tau_in = theoretical_tau_in(x0)
        if tau_in is not None:
            print(f"  theoretical entrance time: tau_in={tau_in:.4f}")


# =========================
# Visualization helpers
# =========================
def add_time_colored_phase(ax, x: np.ndarray, y: np.ndarray, t: np.ndarray, xlabel: str, ylabel: str, title: str):
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

    ax.scatter(x[0], y[0], marker="o", color="green", s=180, label="Start", zorder=5, alpha=0.8)
    ax.scatter(x[-1], y[-1], marker="x", color="red", s=180, label="Finish", zorder=5, alpha=0.8)

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


def save_local_plots(t, xs, us, ws, alive, output_dir: Path):
    valid = valid_mask(xs)
    t_valid = t[valid]
    xs_valid = xs[valid]
    us_valid = us[valid]
    ws_valid = ws[valid]

    plt.figure(figsize=(15, 10), num="Two-Tank Local Control")

    plt.subplot(3, 2, 1)
    plt.plot(t_valid, xs_valid[:, 0], label="h1")
    plt.plot(t_valid, xs_valid[:, 1], label="h2")
    plt.xlim([t[0], t[-1]])
    plt.xlabel("Time (s)")
    plt.ylabel("Level Deviation")
    plt.title("Tank Level Deviations")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 2, 2)
    plt.plot(t_valid, us_valid[:, 0], label="u1")
    plt.plot(t_valid, us_valid[:, 1], label="u2")
    plt.xlim([t[0], t[-1]])
    plt.xlabel("Time (s)")
    plt.ylabel("Control Input")
    plt.title("Local Control Inputs")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 2, 3)
    phase_ax = plt.gca()
    line = add_time_colored_phase(phase_ax, xs_valid[:, 0], xs_valid[:, 1], t_valid, "h1", "h2", "Phase Portrait")
    circle_x, circle_y = phase_boundary_points()
    phase_ax.plot(circle_x, circle_y, "--", color="black", linewidth=1.5, label=r"$W=a$")
    phase_ax.legend()
    if line is not None:
        cb = plt.colorbar(line, ax=phase_ax)
        cb.set_label("Time (s)")

    plt.subplot(3, 2, 4)
    plt.plot(t_valid, ws_valid, label="W(x)")
    plt.axhline(a, color="red", linestyle="--", label="a")
    plt.xlim([t[0], t[-1]])
    plt.xlabel("Time (s)")
    plt.ylabel("W")
    plt.title("Lyapunov Function")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t_valid, state_norm(xs_valid), label=r"$||x||$")
    plt.axhline(np.sqrt(a), color="red", linestyle="--", label=r"$\sqrt{a}$")
    plt.axhline(STATE_MAX, color="black", linestyle=":", label="Runaway Threshold")
    plt.xlim([t[0], t[-1]])
    plt.xlabel("Time (s)")
    plt.ylabel("Norm")
    plt.title("State Norm and Divergence Threshold")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(output_dir / "plots.png")
    plt.close()


def save_hybrid_plots(t, xs, us, ws, region, x0, output_dir: Path):
    valid = valid_mask(xs)
    t_valid = t[valid]
    xs_valid = xs[valid]
    us_valid = us[valid]
    ws_valid = ws[valid]
    tau_in = theoretical_tau_in(x0)

    plt.figure(figsize=(15, 10), num="Two-Tank Hybrid Control")

    plt.subplot(3, 2, 1)
    plt.plot(t_valid, xs_valid[:, 0], label="h1")
    plt.plot(t_valid, xs_valid[:, 1], label="h2")
    plt.xlim([t[0], t[-1]])
    plt.xlabel("Time (s)")
    plt.ylabel("Level Deviation")
    plt.title("Tank Level Deviations")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 2, 2)
    plt.plot(t_valid, us_valid[:, 0], label="u1")
    plt.plot(t_valid, us_valid[:, 1], label="u2")
    plt.xlim([t[0], t[-1]])
    plt.xlabel("Time (s)")
    plt.ylabel("Control Input")
    plt.title("Hybrid Control Inputs")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 2, 3)
    phase_ax = plt.gca()
    line = add_time_colored_phase(phase_ax, xs_valid[:, 0], xs_valid[:, 1], t_valid, "h1", "h2", "Phase Portrait")
    circle_x, circle_y = phase_boundary_points()
    phase_ax.plot(circle_x, circle_y, "--", color="black", linewidth=1.5, label=r"$W=a$")
    phase_ax.legend()
    if line is not None:
        cb = plt.colorbar(line, ax=phase_ax)
        cb.set_label("Time (s)")

    plt.subplot(3, 2, 4)
    plt.plot(t_valid, ws_valid, label="W(x)")
    plt.axhline(a, color="red", linestyle="--", label="a")
    if tau_in is not None:
        plt.axvline(tau_in, color="black", linestyle=":", label=r"$\tau_{in}$")
        outer_mask = region[valid] == 1
        if np.any(outer_mask):
            t_outer = t_valid[outer_mask]
            plt.plot(t_outer, W(x0) * np.exp(-t_outer), linestyle="--", color="tab:green", label=r"$W(x_0)e^{-t}$")
    plt.xlim([t[0], t[-1]])
    plt.xlabel("Time (s)")
    plt.ylabel("W")
    plt.title("Lyapunov Function")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t, region, label="Hybrid Mode")
    plt.xlim([t[0], t[-1]])
    plt.ylim([-0.1, 1.1])
    plt.yticks([0, 1], ["local", "PDE"])
    plt.xlabel("Time (s)")
    plt.ylabel("Mode")
    plt.title("Hybrid Switching Logic")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(output_dir / "plots.png")
    plt.close()


def save_comparison_plots(local_data, hybrid_data, x0: np.ndarray, output_dir: Path):
    t_local, x_local, _, w_local, _, alive_local = local_data
    t_hyb, x_hyb, _, w_hyb, region_hyb, _ = hybrid_data

    valid_local = valid_mask(x_local)
    valid_hyb = valid_mask(x_hyb)
    tau_in = theoretical_tau_in(x0)
    circle_x, circle_y = phase_boundary_points()

    fig, axs = plt.subplots(2, 2, figsize=(14, 10), num="Two-Tank Comparison")

    axs[0, 0].plot(circle_x, circle_y, "--", linewidth=1.5, label=r"$W=a$")
    axs[0, 0].plot(x_local[valid_local, 0], x_local[valid_local, 1], linewidth=2, label="Local only")
    axs[0, 0].plot(x_hyb[valid_hyb, 0], x_hyb[valid_hyb, 1], linewidth=2, label="Hybrid")
    axs[0, 0].scatter([0], [0], marker="x", s=70, label="Target")
    axs[0, 0].scatter([x0[0]], [x0[1]], s=60, label="Start")
    axs[0, 0].set_title("Phase Portrait Comparison")
    axs[0, 0].set_xlabel("h1")
    axs[0, 0].set_ylabel("h2")
    axs[0, 0].axis("equal")
    axs[0, 0].grid(True)
    axs[0, 0].legend()

    axs[0, 1].plot(t_local[valid_local], w_local[valid_local], linewidth=2, label="Local only")
    axs[0, 1].plot(t_hyb[valid_hyb], w_hyb[valid_hyb], linewidth=2, label="Hybrid")
    axs[0, 1].axhline(a, linestyle="--", linewidth=1.5, label=r"$a$")
    if tau_in is not None:
        axs[0, 1].axvline(tau_in, linestyle=":", linewidth=1.5, label=r"$\tau_{in}$")
    axs[0, 1].set_title(r"Lyapunov Function $W(h)=h_1^2+h_2^2$")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("W")
    axs[0, 1].grid(True)
    axs[0, 1].legend()

    axs[1, 0].plot(t_local[valid_local], x_local[valid_local, 0], linewidth=2, label=r"$h_1$ local")
    axs[1, 0].plot(t_local[valid_local], x_local[valid_local, 1], linewidth=2, label=r"$h_2$ local")
    axs[1, 0].plot(t_hyb[valid_hyb], x_hyb[valid_hyb, 0], linewidth=2, linestyle="--", label=r"$h_1$ hybrid")
    axs[1, 0].plot(t_hyb[valid_hyb], x_hyb[valid_hyb, 1], linewidth=2, linestyle="--", label=r"$h_2$ hybrid")
    axs[1, 0].set_title("Tank Level Deviations")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Level Deviation")
    axs[1, 0].grid(True)
    axs[1, 0].legend()

    axs[1, 1].plot(t_hyb, region_hyb, linewidth=2)
    axs[1, 1].set_title("Hybrid Mode Indicator")
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].set_ylabel("Mode")
    axs[1, 1].set_yticks([0, 1])
    axs[1, 1].set_yticklabels(["local", "PDE"])
    axs[1, 1].grid(True)

    if not alive_local[-1]:
        axs[0, 1].text(
            0.02,
            0.92,
            "Local trajectory diverges and is clipped",
            transform=axs[0, 1].transAxes,
            fontsize=10,
            bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.8},
        )

    plt.tight_layout()
    plt.savefig(output_dir / "plots.png")
    plt.close(fig)


def create_tank_animation(
    t: np.ndarray,
    xs: np.ndarray,
    region: np.ndarray,
    output_path: Path,
    title: str,
    mode_label: str,
    comparison_xs: Optional[np.ndarray] = None,
    comparison_valid: Optional[np.ndarray] = None,
):
    valid = valid_mask(xs)
    circle_x, circle_y = phase_boundary_points()

    fig = plt.figure(figsize=(13, 6))
    gs = fig.add_gridspec(1, 2, width_ratios=[1, 1.2])

    ax_tanks = fig.add_subplot(gs[0, 0])
    ax_phase = fig.add_subplot(gs[0, 1])

    tank_width = 0.8
    tank_gap = 0.8
    tank_x = [0.0, tank_width + tank_gap]

    ax_tanks.set_xlim(-0.3, tank_x[1] + tank_width + 0.3)
    ax_tanks.set_ylim(0.0, TANK_HEIGHT + 0.7)
    ax_tanks.set_aspect("equal")
    ax_tanks.set_title(title)
    ax_tanks.set_xticks([])
    ax_tanks.set_ylabel("Water Level")

    for x_left in tank_x:
        ax_tanks.plot([x_left, x_left], [0.0, TANK_HEIGHT], "k-", linewidth=2)
        ax_tanks.plot([x_left + tank_width, x_left + tank_width], [0.0, TANK_HEIGHT], "k-", linewidth=2)
        ax_tanks.plot([x_left, x_left + tank_width], [0.0, 0.0], "k-", linewidth=2)

    water1 = plt.Rectangle((tank_x[0], 0.0), tank_width, NOMINAL_LEVEL, alpha=0.55, color="royalblue")
    water2 = plt.Rectangle((tank_x[1], 0.0), tank_width, NOMINAL_LEVEL, alpha=0.55, color="crimson")
    ax_tanks.add_patch(water1)
    ax_tanks.add_patch(water2)

    tank_text = ax_tanks.text(0.02, 0.98, "", transform=ax_tanks.transAxes, va="top")
    mode_text = ax_tanks.text(0.02, 0.88, "", transform=ax_tanks.transAxes, va="top")

    all_pts = [xs[valid]]
    if comparison_xs is not None and comparison_valid is not None and np.any(comparison_valid):
        all_pts.append(comparison_xs[comparison_valid])
    lim = max(2.5, np.max(np.abs(np.vstack(all_pts))) + 0.5)

    ax_phase.set_xlim(-lim, lim)
    ax_phase.set_ylim(-lim, lim)
    ax_phase.set_aspect("equal")
    ax_phase.grid(True)
    ax_phase.set_title("Phase Portrait and Switching Boundary")
    ax_phase.set_xlabel("h1")
    ax_phase.set_ylabel("h2")
    ax_phase.plot(circle_x, circle_y, "--", linewidth=1.5, label=r"$W=a$")
    ax_phase.scatter([0], [0], marker="x", s=60, label="Target")

    line_main, = ax_phase.plot([], [], linewidth=2, label=mode_label)
    point_main, = ax_phase.plot([], [], "o", markersize=8)

    line_cmp = point_cmp = None
    if comparison_xs is not None:
        line_cmp, = ax_phase.plot([], [], linewidth=2, linestyle="--", color="tab:orange", label="Local only")
        point_cmp, = ax_phase.plot([], [], "o", markersize=8, color="tab:orange")

    ax_phase.legend(loc="upper right")

    def init():
        water1.set_height(NOMINAL_LEVEL)
        water2.set_height(NOMINAL_LEVEL)
        line_main.set_data([], [])
        point_main.set_data([], [])
        if line_cmp is not None:
            line_cmp.set_data([], [])
            point_cmp.set_data([], [])
        tank_text.set_text("")
        mode_text.set_text("")

        artists = [water1, water2, line_main, point_main, tank_text, mode_text]
        if line_cmp is not None:
            artists.extend([line_cmp, point_cmp])
        return tuple(artists)

    def update(frame: int):
        h1_value, h2_value = xs[frame]
        level1 = clamp_level(h1_value if np.isfinite(h1_value) else 0.0)
        level2 = clamp_level(h2_value if np.isfinite(h2_value) else 0.0)
        water1.set_height(level1)
        water2.set_height(level2)

        if np.isfinite(xs[frame, 0]) and np.isfinite(xs[frame, 1]):
            line_main.set_data(xs[: frame + 1, 0], xs[: frame + 1, 1])
            point_main.set_data([xs[frame, 0]], [xs[frame, 1]])

        if comparison_xs is not None and comparison_valid is not None and line_cmp is not None:
            cmp_mask = comparison_valid[: frame + 1]
            if np.any(cmp_mask):
                idx = np.where(cmp_mask)[0]
                line_cmp.set_data(comparison_xs[idx, 0], comparison_xs[idx, 1])
                point_cmp.set_data([comparison_xs[idx[-1], 0]], [comparison_xs[idx[-1], 1]])

        current_mode = "PDE outer control" if region[frame] == 1 else "local inner control"
        tank_text.set_text(f"Time = {t[frame]:.2f}s\nh1 = {xs[frame, 0]:.2f}, h2 = {xs[frame, 1]:.2f}")
        mode_text.set_text(f"Mode = {current_mode}")

        artists = [water1, water2, line_main, point_main, tank_text, mode_text]
        if line_cmp is not None:
            artists.extend([line_cmp, point_cmp])
        return tuple(artists)

    step = max(1, len(t) // 240)
    animation = FuncAnimation(
        fig,
        update,
        frames=range(0, len(t), step),
        interval=30,
        blit=True,
        init_func=init,
    )

    animation.save(output_path, writer="pillow", fps=24)
    plt.close(fig)
