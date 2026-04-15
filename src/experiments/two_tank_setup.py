from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from controllers.hybrid import HybridController
from controllers.local import LocalLinearController
from controllers.outer import PDEOuterController
from controllers.swirl import SwirlOuterController
from core.interfaces import Controller
from core.types import Array
from lyapunov.quadratic import (
    QuadraticLyapunovData,
    QuadraticLyapunovFunction,
    build_local_lyapunov_data,
)
from simulation.simulator import (
    ClosedLoopSimulator,
    SimulationConfig,
    SimulationResult,
)
from systems.two_tank import TwoTankParameters, TwoTankSystem

DEFAULT_X0 = np.array([10.0, 10.0], dtype=float)
DEFAULT_CONFIG = SimulationConfig(t0=0.0, tf=18.0, dt=0.01, state_max_norm=20.0)


@dataclass(frozen=True)
class TwoTankHybridSetup:
    system: TwoTankSystem
    lyapunov_data: QuadraticLyapunovData
    lyapunov: QuadraticLyapunovFunction
    local_controller: LocalLinearController
    outer_controller: Controller
    hybrid_controller: HybridController


def build_two_tank_setup(
    alpha: float = 2.4,
    kappa: float = 0.8,
    r: float = 1.0,
    k: float = 1.2,
    sigma_level: float = 0.8,
    use_swirl: bool = False,
    swirl_gain: float = 2.0,
) -> TwoTankHybridSetup:
    system = TwoTankSystem(TwoTankParameters(alpha=alpha, kappa=kappa, r=r))

    K = -k * np.eye(2, dtype=float)
    lyap_data = build_local_lyapunov_data(system, K)
    lyapunov = QuadraticLyapunovFunction(lyap_data.P)

    local_controller = LocalLinearController(lyap_data.K)
    base_outer = PDEOuterController(
        system=system,
        lyapunov=lyapunov,
        sigma_level=sigma_level,
    )

    if use_swirl:
        outer_controller: Controller = SwirlOuterController(
            base_outer=base_outer,
            swirl_gain=swirl_gain,
        )
    else:
        outer_controller = base_outer

    hybrid_controller = HybridController(
        lyapunov=lyapunov,
        sigma_level=sigma_level,
        local_controller=local_controller,
        outer_controller=outer_controller,
    )

    return TwoTankHybridSetup(
        system=system,
        lyapunov_data=lyap_data,
        lyapunov=lyapunov,
        local_controller=local_controller,
        outer_controller=outer_controller,
        hybrid_controller=hybrid_controller,
    )


def run_two_tank_experiment(
    x0: Array = DEFAULT_X0,
    config: SimulationConfig = DEFAULT_CONFIG,
    alpha: float = 2.4,
    kappa: float = 0.8,
    r: float = 1.0,
    k: float = 1.2,
    sigma_level: float = 0.8,
    use_swirl: bool = False,
    swirl_gain: float = 10.0,
) -> tuple[TwoTankHybridSetup, SimulationResult]:
    setup = build_two_tank_setup(
        alpha=alpha,
        kappa=kappa,
        r=r,
        k=k,
        sigma_level=sigma_level,
        use_swirl=use_swirl,
        swirl_gain=swirl_gain,
    )
    simulator = ClosedLoopSimulator(
        system=setup.system,
        controller=setup.hybrid_controller,
        lyapunov=setup.lyapunov,
        config=config,
    )
    return setup, simulator.simulate(x0)
