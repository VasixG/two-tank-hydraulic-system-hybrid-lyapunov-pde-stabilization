import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from common import simulate, W, alpha, kappa, r, k, a, dt, N

def main():
    # initial condition
    x0 = np.array([2.2, 1.8], dtype=float)

    # Simulate hybrid control
    t, xs, us, ws, region, alive = simulate(x0, "hybrid")

    tau_in_theory = None
    if W(x0) > a:
        tau_in_theory = np.log(W(x0) / a)

    print("Parameters")
    print(f"alpha={alpha}, kappa={kappa}, r={r}, k={k}, a={a}")
    print(f"x0={x0}, W(x0)={W(x0):.4f}")
    if tau_in_theory is not None:
        print(f"Theoretical entrance time: tau_in={tau_in_theory:.4f}")

    # Static plots
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))

    # ---- Phase portrait
    theta = np.linspace(0, 2*np.pi, 400)
    radius = np.sqrt(a)
    circle_x = radius * np.cos(theta)
    circle_y = radius * np.sin(theta)

    valid = np.isfinite(xs[:, 0]) & np.isfinite(xs[:, 1])

    axs[0, 0].plot(circle_x, circle_y, "--", linewidth=1.5, label=r"$W(h)=a$")
    axs[0, 0].plot(xs[valid, 0], xs[valid, 1], linewidth=2, label="Hybrid control")
    axs[0, 0].scatter([0], [0], marker="x", s=70, label="Target")
    axs[0, 0].scatter([x0[0]], [x0[1]], s=60, label="Start")
    axs[0, 0].set_title("Phase portrait")
    axs[0, 0].set_xlabel(r"$h_1$")
    axs[0, 0].set_ylabel(r"$h_2$")
    axs[0, 0].axis("equal")
    axs[0, 0].grid(True)
    axs[0, 0].legend()

    # ---- W(t)
    axs[0, 1].plot(t[valid], ws[valid], linewidth=2, label="Hybrid control")
    axs[0, 1].axhline(a, linestyle="--", linewidth=1.5, label=r"$a$")
    if tau_in_theory is not None:
        axs[0, 1].axvline(tau_in_theory, linestyle=":", linewidth=1.5, label=r"$\tau_{\rm in}$")
    axs[0, 1].set_title(r"Lyapunov function $W(h)=h_1^2+h_2^2$")
    axs[0, 1].set_xlabel("t")
    axs[0, 1].set_ylabel("W")
    axs[0, 1].grid(True)
    axs[0, 1].legend()

    # ---- Levels over time
    axs[1, 0].plot(t[valid], xs[valid, 0], linewidth=2, label=r"$h_1$")
    axs[1, 0].plot(t[valid], xs[valid, 1], linewidth=2, label=r"$h_2$")
    axs[1, 0].set_title("Tank level deviations")
    axs[1, 0].set_xlabel("t")
    axs[1, 0].set_ylabel("level deviation")
    axs[1, 0].grid(True)
    axs[1, 0].legend()

    # ---- Hybrid mode indicator
    axs[1, 1].plot(t, region, linewidth=2)
    axs[1, 1].set_title("Hybrid mode indicator")
    axs[1, 1].set_xlabel("t")
    axs[1, 1].set_ylabel("mode")
    axs[1, 1].set_yticks([0, 1])
    axs[1, 1].set_yticklabels(["local", "PDE"])
    axs[1, 1].grid(True)

    plt.tight_layout()
    plt.savefig("gfx/hybrid_control/plots.png")
    plt.show()

    # Animation
    fig_anim = plt.figure(figsize=(13, 6))
    gs = fig_anim.add_gridspec(1, 2, width_ratios=[1, 1.2])

    ax_tanks = fig_anim.add_subplot(gs[0, 0])
    ax_phase = fig_anim.add_subplot(gs[0, 1])

    # Tank drawing setup
    tank_width = 0.8
    tank_gap = 0.8
    tank_x = [0.0, tank_width + tank_gap]
    tank_bottom = 0.0
    tank_height = 6.0

    nominal_level = 2.5
    display_min = 0.0
    display_max = tank_height

    ax_tanks.set_xlim(-0.3, tank_x[1] + tank_width + 0.3)
    ax_tanks.set_ylim(0.0, tank_height + 0.7)
    ax_tanks.set_aspect("equal")
    ax_tanks.set_title("Two-tank visualization")
    ax_tanks.set_xticks([])
    ax_tanks.set_ylabel("water level")

    # tank walls
    for x_left in tank_x:
        ax_tanks.plot([x_left, x_left], [tank_bottom, tank_height], "k-", linewidth=2)
        ax_tanks.plot([x_left + tank_width, x_left + tank_width], [tank_bottom, tank_height], "k-", linewidth=2)
        ax_tanks.plot([x_left, x_left + tank_width], [tank_bottom, tank_bottom], "k-", linewidth=2)

    # water rectangles
    water1 = plt.Rectangle((tank_x[0], tank_bottom), tank_width, nominal_level, alpha=0.5)
    water2 = plt.Rectangle((tank_x[1], tank_bottom), tank_width, nominal_level, alpha=0.5)
    ax_tanks.add_patch(water1)
    ax_tanks.add_patch(water2)

    # text labels
    tank_text = ax_tanks.text(0.02, 0.98, "", transform=ax_tanks.transAxes, va="top")
    mode_text = ax_tanks.text(0.02, 0.90, "", transform=ax_tanks.transAxes, va="top")

    # Phase portrait setup
    all_pts = xs[valid]
    lim = max(2.5, np.max(np.abs(all_pts)) + 0.5)

    ax_phase.set_xlim(-lim, lim)
    ax_phase.set_ylim(-lim, lim)
    ax_phase.set_aspect("equal")
    ax_phase.grid(True)
    ax_phase.set_title("Phase portrait and switching boundary")
    ax_phase.set_xlabel(r"$h_1$")
    ax_phase.set_ylabel(r"$h_2$")

    ax_phase.plot(circle_x, circle_y, "--", linewidth=1.5, label=r"$W=a$")
    ax_phase.scatter([0], [0], marker="x", s=60, label="Target")

    line_hyb, = ax_phase.plot([], [], linewidth=2, label="Hybrid")
    point_hyb, = ax_phase.plot([], [], "o", markersize=8)
    ax_phase.legend(loc="upper right")

    def clamp_level(h: float) -> float:
        return float(np.clip(nominal_level + h, display_min, display_max))

    def init():
        water1.set_height(nominal_level)
        water2.set_height(nominal_level)
        line_hyb.set_data([], [])
        point_hyb.set_data([], [])
        tank_text.set_text("")
        mode_text.set_text("")
        return water1, water2, line_hyb, point_hyb, tank_text, mode_text

    def update(frame: int):
        h1_h, h2_h = xs[frame]
        level1 = clamp_level(h1_h if np.isfinite(h1_h) else 0.0)
        level2 = clamp_level(h2_h if np.isfinite(h2_h) else 0.0)
        water1.set_height(level1)
        water2.set_height(level2)

        # phase history
        if np.isfinite(xs[frame, 0]) and np.isfinite(xs[frame, 1]):
            line_hyb.set_data(xs[:frame+1, 0], xs[:frame+1, 1])
            point_hyb.set_data([xs[frame, 0]], [xs[frame, 1]])

        current_mode = "PDE outer control" if region[frame] == 1 else "local inner control"
        tank_text.set_text(
            f"t = {frame*dt:.2f}\n"
            f"h1 = {xs[frame, 0]:.2f}, h2 = {xs[frame, 1]:.2f}"
        )
        mode_text.set_text(f"mode: {current_mode}")

        return water1, water2, line_hyb, point_hyb, tank_text, mode_text

    ani = FuncAnimation(
        fig_anim,
        update,
        frames=N + 1,
        init_func=init,
        interval=25,
        blit=True
    )

    plt.tight_layout()
    plt.show()

    # To save animation
    # ani.save("gfx/hybrid_control/animation.gif", writer="pillow", fps=30)
    # ani.save("gfx/hybrid_control/animation.mp4", writer="ffmpeg", fps=30)

if __name__ == "__main__":
    main()