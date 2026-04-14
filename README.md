# Two-Tank Hydraulic System Hybrid Lyapunov PDE Stabilization

Hybrid control for a two-tank hydraulic system using Lyapunov function and PDE-based stabilization.

## Description

This project implements a hybrid stabilization strategy for a nonlinear two-tank hydraulic system. The system consists of two coupled tanks with destabilizing nonlinear inflows and inter-tank coupling. The control strategy combines:

1. **Local Linear Control**: Fast stabilization near the equilibrium using linear feedback
2. **PDE-Based Outer Control**: Robust stabilization in the outer region using Lyapunov level-set methods

The hybrid law switches between these controls based on the Lyapunov function value, ensuring global asymptotic stability.

## Installation

```bash
pip install -r requirements.txt
```

## Usage

Local control only:

```bash
python two_tank_local.py
```

Hybrid control (recommended):

```bash
python two_tank_hybrid.py
```

## Mathematical Background

The system dynamics are given by:

$$\dot{x} = f(x) + u$$

where $x = [h_1, h_2]^\top$ are level deviations, and $f(x)$ includes nonlinear terms and coupling.

The Lyapunov function is $W(x) = h_1^2 + h_2^2$.

See [README-derivation.md](README-derivation.md) for complete mathematical derivations.

## Results

✅ **What Works Well**:
- Successful stabilization from large initial conditions
- Smooth switching between control modes
- Finite-time convergence to the inner region

⚠️ **Current Limitations**:
- Local control may diverge for very large initial conditions
- Parameter sensitivity

## Project Structure

- `common.py`: Shared model functions and simulation utilities
- `two_tank_local.py`: Local linear control simulation
- `two_tank_hybrid.py`: Hybrid control simulation with animation
- `README-derivation.md`: Detailed mathematical proofs
- `gfx/`: Generated plots and animations (created at runtime)

## Dependencies

- numpy
- matplotlib

## License

MIT