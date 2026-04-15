from __future__ import annotations

import numpy as np

from core.interfaces import Controller
from core.types import Array, as_float_array


class LocalLinearController(Controller):
    def __init__(self, K: Array):
        self.K = np.asarray(K, dtype=float)

    def control(self, x: Array) -> Array:
        x = as_float_array(x)
        return self.K @ x
