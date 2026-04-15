from __future__ import annotations

import numpy as np

from core.interfaces import ControlAffineSystem, Controller
from core.types import Array, as_float_array
from lyapunov.quadratic import QuadraticLyapunovFunction


class PDEOuterController(Controller):
    def __init__(
        self,
        system: ControlAffineSystem,
        lyapunov: QuadraticLyapunovFunction,
        sigma_level: float,
        regularization_eps: float = 1e-12,
    ):
        if sigma_level <= 0.0:
            raise ValueError("sigma_level must be positive.")

        self.system = system
        self.lyapunov = lyapunov
        self.sigma_level = float(sigma_level)
        self.regularization_eps = float(regularization_eps)

    def W(self, x: Array) -> float:
        return self.lyapunov.value(x)

    def grad_W(self, x: Array) -> Array:
        return self.lyapunov.grad(x)

    def T(self, x: Array) -> float:
        w = self.W(x)
        if w <= self.sigma_level:
            raise ValueError("T is only defined in the outer region W > a.")
        return float(np.log(w / self.sigma_level))

    def grad_T(self, x: Array) -> Array:
        w = self.W(x)
        if w <= self.sigma_level:
            raise ValueError("grad_T is only defined in the outer region W > a.")
        return self.grad_W(x) / w

    def control(self, x: Array) -> Array:
        x = as_float_array(x)
        w = self.W(x)
        if w <= self.sigma_level:
            raise ValueError("Outer controller called inside the inner region.")

        gW = self.grad_W(x)
        Gx = self.system.G(x)
        fx = self.system.f(x)

        b = Gx.T @ gW
        denom = float(b @ b)

        if denom <= self.regularization_eps:
            raise ValueError("Outer control is undefined: G(x)^T grad W(x) is too small.")

        scalar = -(w + float(gW @ fx)) / denom
        return scalar * b

    def pde_residual(self, x: Array, u: Array | None = None) -> float:
        x = as_float_array(x)
        if u is None:
            u = self.control(x)
        else:
            u = as_float_array(u)

        gT = self.grad_T(x)
        return float(gT @ (self.system.f(x) + self.system.G(x) @ u) + 1.0)
