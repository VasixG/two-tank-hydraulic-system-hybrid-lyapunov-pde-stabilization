import numpy as np
import matplotlib.pyplot as plt
from common import simulate, W, alpha, kappa, r, k, a, dt, N

def main():
    # initial condition
    x0 = np.array([2.2, 1.8], dtype=float)

    # Simulate local control
    t, xs, us, ws, region, alive = simulate(x0, "local")

    print("Parameters")
    print(f"alpha={alpha}, kappa={kappa}, r={r}, k={k}, a={a}")
    print(f"x0={x0}, W(x0)={W(x0):.4f}")

    # Static plots
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))

    # ---- Phase portrait
    theta = np.linspace(0, 2*np.pi, 400)
    radius = np.sqrt(a)
    circle_x = radius * np.cos(theta)
    circle_y = radius * np.sin(theta)

    valid = np.isfinite(xs[:, 0]) & np.isfinite(xs[:, 1])

    axs[0, 0].plot(circle_x, circle_y, "--", linewidth=1.5, label=r"$W(h)=a$")
    axs[0, 0].plot(xs[valid, 0], xs[valid, 1], linewidth=2, label="Local control")
    axs[0, 0].scatter([0], [0], marker="x", s=70, label="Target")
    axs[0, 0].scatter([x0[0]], [x0[1]], s=60, label="Start")
    axs[0, 0].set_title("Phase portrait")
    axs[0, 0].set_xlabel(r"$h_1$")
    axs[0, 0].set_ylabel(r"$h_2$")
    axs[0, 0].axis("equal")
    axs[0, 0].grid(True)
    axs[0, 0].legend()

    # ---- W(t)
    axs[0, 1].plot(t[valid], ws[valid], linewidth=2, label="Local control")
    axs[0, 1].axhline(a, linestyle="--", linewidth=1.5, label=r"$a$")
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

    # ---- Control inputs
    axs[1, 1].plot(t[valid], us[valid, 0], linewidth=2, label=r"$u_1$")
    axs[1, 1].plot(t[valid], us[valid, 1], linewidth=2, label=r"$u_2$")
    axs[1, 1].set_title("Control inputs")
    axs[1, 1].set_xlabel("t")
    axs[1, 1].set_ylabel("u")
    axs[1, 1].grid(True)
    axs[1, 1].legend()

    plt.tight_layout()
    plt.savefig("gfx/local_control/plots.png")
    plt.show()

if __name__ == "__main__":
    main()