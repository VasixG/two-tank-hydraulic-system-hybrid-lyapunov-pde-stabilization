from __future__ import annotations

import numpy as np

from controllers.outer import PDEOuterController
from core.interfaces import Controller
from core.types import Array, as_float_array


class SwirlOuterController(Controller):
    def __init__(
        self,
        base_outer: PDEOuterController,
        swirl_gain: float,
        omega_eps: float = 1e-12,
    ):
        self.base_outer = base_outer
        self.swirl_gain = float(swirl_gain)
        self.omega_eps = float(omega_eps)

        if self.base_outer.system.state_dim != 2:
            raise ValueError("Swirl controller is implemented only for 2D state spaces.")

    def omega(self, x: Array) -> float:
        w = self.base_outer.W(x)
        a = self.base_outer.sigma_level
        return self.swirl_gain * max(w - a, 0.0) / (w + self.omega_eps)

    def tangential_direction(self, x: Array) -> Array:
        gT = self.base_outer.grad_T(x)
        return np.array([-gT[1], gT[0]], dtype=float)

    def control(self, x: Array) -> Array:
        x = as_float_array(x)
        u_base = self.base_outer.control(x)
        u_swirl = self.omega(x) * self.tangential_direction(x)
        return u_base + u_swirl

    def pde_residual(self, x: Array, u: Array | None = None) -> float:
        x = as_float_array(x)
        if u is None:
            u = self.control(x)
        else:
            u = as_float_array(u)

        gT = self.base_outer.grad_T(x)
        system = self.base_outer.system
        return float(gT @ (system.f(x) + system.G(x) @ u) + 1.0)
