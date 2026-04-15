from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from controllers.hybrid import HybridController
from core.interfaces import ControlAffineSystem
from core.types import Array, as_float_array
from lyapunov.quadratic import QuadraticLyapunovFunction
from simulation.rk4 import rk4_step


@dataclass(frozen=True)
class SimulationConfig:
    t0: float = 0.0
    tf: float = 10.0
    dt: float = 0.01
    state_max_norm: float | None = None


@dataclass(frozen=True)
class SimulationResult:
    t: Array
    x: Array
    u: Array
    W: Array
    mode: list[str]
    alive: bool


class ClosedLoopSimulator:
    def __init__(
        self,
        system: ControlAffineSystem,
        controller: HybridController,
        lyapunov: QuadraticLyapunovFunction,
        config: SimulationConfig,
    ):
        self.system = system
        self.controller = controller
        self.lyapunov = lyapunov
        self.config = config

    def rhs(self, t: float, x: Array) -> Array:
        _ = t
        u = self.controller.control(x)
        return self.system.dynamics(x, u)

    def simulate(self, x0: Array) -> SimulationResult:
        x0 = as_float_array(x0)

        t0 = self.config.t0
        tf = self.config.tf
        dt = self.config.dt
        n_steps = int(np.floor((tf - t0) / dt)) + 1

        t = np.linspace(t0, t0 + dt * (n_steps - 1), n_steps)
        x_hist = np.full((n_steps, self.system.state_dim), np.nan, dtype=float)
        u_hist = np.full((n_steps, self.system.control_dim), np.nan, dtype=float)
        W_hist = np.full(n_steps, np.nan, dtype=float)
        mode_hist: list[str] = []

        x = x0.copy()
        alive = True

        for k in range(n_steps):
            if not np.all(np.isfinite(x)):
                alive = False
                break

            if self.config.state_max_norm is not None:
                if np.linalg.norm(x) > self.config.state_max_norm:
                    alive = False
                    break

            x_hist[k] = x
            u = self.controller.control(x)
            u_hist[k] = u
            W_hist[k] = self.lyapunov.value(x)
            mode_hist.append(self.controller.mode(x))

            if k < n_steps - 1:
                x = rk4_step(self.rhs, t[k], x, dt)

        if len(mode_hist) < n_steps:
            mode_hist.extend(["terminated"] * (n_steps - len(mode_hist)))

        return SimulationResult(
            t=t,
            x=x_hist,
            u=u_hist,
            W=W_hist,
            mode=mode_hist,
            alive=alive,
        )
