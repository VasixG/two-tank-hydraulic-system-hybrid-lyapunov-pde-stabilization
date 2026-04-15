from __future__ import annotations

from abc import ABC, abstractmethod

from core.types import Array, as_float_array


class ControlAffineSystem(ABC):
    @property
    @abstractmethod
    def state_dim(self) -> int:
        pass

    @property
    @abstractmethod
    def control_dim(self) -> int:
        pass

    @abstractmethod
    def f(self, x: Array) -> Array:
        """Drift vector field."""
        pass

    @abstractmethod
    def G(self, x: Array) -> Array:
        """Input matrix."""
        pass

    @abstractmethod
    def linearization(self) -> tuple[Array, Array]:
        """Return (A, B) at the equilibrium x = 0."""
        pass

    def dynamics(self, x: Array, u: Array) -> Array:
        x = as_float_array(x)
        u = as_float_array(u)
        return self.f(x) + self.G(x) @ u


class Controller(ABC):
    @abstractmethod
    def control(self, x: Array) -> Array:
        pass
