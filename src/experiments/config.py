from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from core.types import Array
from simulation.simulator import SimulationConfig

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "default.json"


@dataclass(frozen=True)
class ExperimentConfig:
    alpha: float
    kappa: float
    r: float
    k: float
    sigma_level: float
    swirl_gain: float
    x0: Array
    simulation: SimulationConfig


def load_experiment_config(path: Path = DEFAULT_CONFIG_PATH) -> ExperimentConfig:
    if not path.exists():
        return ExperimentConfig(
            alpha=2.4,
            kappa=0.8,
            r=1.0,
            k=1.2,
            sigma_level=0.8,
            swirl_gain=10.0,
            x0=np.array([10.0, 10.0], dtype=float),
            simulation=SimulationConfig(t0=0.0, tf=18.0, dt=0.01, state_max_norm=20.0),
        )

    with path.open("r", encoding="utf-8") as file:
        raw = json.load(file)

    system = raw.get("system", {})
    controller = raw.get("controller", {})
    simulation = raw.get("simulation", {})

    return ExperimentConfig(
        alpha=float(system.get("alpha", 2.4)),
        kappa=float(system.get("kappa", 0.8)),
        r=float(system.get("r", 1.0)),
        k=float(controller.get("k", 1.2)),
        sigma_level=float(controller.get("sigma_level", 0.8)),
        swirl_gain=float(controller.get("swirl_gain", 10.0)),
        x0=np.asarray(simulation.get("x0", [10.0, 10.0]), dtype=float),
        simulation=SimulationConfig(
            t0=float(simulation.get("t0", 0.0)),
            tf=float(simulation.get("tf", 18.0)),
            dt=float(simulation.get("dt", 0.01)),
            state_max_norm=simulation.get("state_max_norm", 20.0),
        ),
    )
