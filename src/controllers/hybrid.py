from __future__ import annotations

from core.interfaces import Controller
from core.types import Array, as_float_array
from lyapunov.quadratic import QuadraticLyapunovFunction


class HybridController(Controller):
    def __init__(
        self,
        lyapunov: QuadraticLyapunovFunction,
        sigma_level: float,
        local_controller: Controller,
        outer_controller: Controller,
    ):
        if sigma_level <= 0.0:
            raise ValueError("sigma_level must be positive.")

        self.lyapunov = lyapunov
        self.sigma_level = float(sigma_level)
        self.local_controller = local_controller
        self.outer_controller = outer_controller

    def W(self, x: Array) -> float:
        return self.lyapunov.value(x)

    def in_outer_region(self, x: Array) -> bool:
        return self.W(x) > self.sigma_level

    def control(self, x: Array) -> Array:
        x = as_float_array(x)
        if self.in_outer_region(x):
            return self.outer_controller.control(x)
        return self.local_controller.control(x)

    def mode(self, x: Array) -> str:
        return "outer" if self.in_outer_region(x) else "local"
