import numpy as np

# =========================
# Parameters
# =========================
alpha = 2.4   # intensity of destabilizing nonlinear inflow effect
kappa = 0.8   # coupling between tanks
r = 1.0       # saturation scale in nonlinear term
k = 1.2       # local linear feedback gain
a = 0.8       # switching threshold: W = h1^2 + h2^2 <= a

# simulation settings
dt = 0.01
T = 18.0
N = int(T / dt)

# clipping threshold for runaway local trajectory
STATE_MAX = 8.0

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
    return float(np.dot(x, x))

def u_loc(x: np.ndarray) -> np.ndarray:
    """Local linear stabilizer."""
    return -k * x

def u_ext(x: np.ndarray) -> np.ndarray:
    """
    External PDE-based control for G = I and W = ||x||^2.
    Formula:
        u_ext = -(W + gradW^T f) / ||gradW||^2 * gradW
    with gradW = 2x.
    """
    w = W(x)
    if w < 1e-12:
        return np.zeros_like(x)

    gradW = 2.0 * x
    num = w + float(np.dot(gradW, f(x)))
    den = float(np.dot(gradW, gradW))
    return -(num / den) * gradW

def u_hybrid(x: np.ndarray) -> np.ndarray:
    """Hybrid law: outer PDE control, inner local control."""
    if W(x) > a:
        return u_ext(x)
    return u_loc(x)

def rhs(x: np.ndarray, mode: str) -> np.ndarray:
    if mode == "local":
        u = u_loc(x)
    elif mode == "hybrid":
        u = u_hybrid(x)
    else:
        raise ValueError("mode must be 'local' or 'hybrid'")
    return f(x) + u

# =========================
# Integrator
# =========================
def rk4_step(x: np.ndarray, dt: float, mode: str) -> np.ndarray:
    k1 = rhs(x, mode)
    k2 = rhs(x + 0.5 * dt * k1, mode)
    k3 = rhs(x + 0.5 * dt * k2, mode)
    k4 = rhs(x + dt * k3, mode)
    return x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

def simulate(x0: np.ndarray, mode: str):
    xs = np.full((N + 1, 2), np.nan)
    us = np.full((N + 1, 2), np.nan)
    ws = np.full(N + 1, np.nan)
    region = np.zeros(N + 1, dtype=int)  # 0=inner(local), 1=outer(PDE)
    alive = np.ones(N + 1, dtype=bool)

    x = x0.copy()
    xs[0] = x
    ws[0] = W(x)
    us[0] = u_loc(x) if mode == "local" else u_hybrid(x)
    region[0] = int(ws[0] > a) if mode == "hybrid" else 0

    for i in range(N):
        if not np.all(np.isfinite(x)) or np.max(np.abs(x)) > STATE_MAX:
            alive[i + 1:] = False
            break

        x_new = rk4_step(x, dt, mode)

        if not np.all(np.isfinite(x_new)) or np.max(np.abs(x_new)) > STATE_MAX:
            alive[i + 1:] = False
            break

        x = x_new
        xs[i + 1] = x
        ws[i + 1] = W(x)
        us[i + 1] = u_loc(x) if mode == "local" else u_hybrid(x)
        region[i + 1] = int(ws[i + 1] > a) if mode == "hybrid" else 0

    t = np.linspace(0.0, N * dt, N + 1)
    return t, xs, us, ws, region, alive