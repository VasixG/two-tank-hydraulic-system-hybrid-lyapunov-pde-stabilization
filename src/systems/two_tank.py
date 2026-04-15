from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from core.interfaces import ControlAffineSystem
from core.types import Array, as_float_array


@dataclass(frozen=True)
class TwoTankParameters:
    alpha: float = 2.4
    kappa: float = 0.8
    r: float = 1.0


class TwoTankSystem(ControlAffineSystem):
    def __init__(self, params: TwoTankParameters):
        self.params = params

    @property
    def state_dim(self) -> int:
        return 2

    @property
    def control_dim(self) -> int:
        return 2

    def f(self, x: Array) -> Array:
        x = as_float_array(x)
        if x.shape[0] != 2:
            raise ValueError("TwoTankSystem expects a 2D state.")

        h1, h2 = x
        alpha = self.params.alpha
        kappa = self.params.kappa
        r = self.params.r

        nl1 = alpha * (h1**2 / (r**2 + h1**2)) * h1
        nl2 = alpha * (h2**2 / (r**2 + h2**2)) * h2
        c12 = kappa * (h2 - h1)
        c21 = kappa * (h1 - h2)

        return np.array([nl1 + c12, nl2 + c21], dtype=float)

    def G(self, x: Array) -> Array:
        _ = x
        return np.eye(2, dtype=float)

    def linearization(self) -> tuple[Array, Array]:
        kappa = self.params.kappa
        A = np.array(
            [
                [-kappa, kappa],
                [kappa, -kappa],
            ],
            dtype=float,
        )
        B = np.eye(2, dtype=float)
        return A, B
