# Two-Tank Hydraulic System Hybrid Lyapunov-PDE Control

This project studies global stabilization of a nonlinear two-tank hydraulic system by a hybrid controller. The inner controller is a local linear stabilizer. The outer controller is a Lyapunov-PDE feedback that drives the state to the inner Lyapunov sublevel set in finite time. A second outer controller adds a tangential swirl term while preserving the same Lyapunov decay law.

<p align="center">
  <img src="animations/idea.gif" alt="two-tank hybrid control idea" width="700">
</p>
<p align="center">
  <em>Concept animation: the outer controller drives the state toward the switching boundary, then the local controller stabilizes the origin.</em>
</p>

The implementation is organized as an object-oriented `src/` layout: the plant, controllers, Lyapunov functions, simulator, and visualization code are separated into modules.

## Problem Definition

The control task is to stabilize the equilibrium

```math
x^\star = 0
```

of a nonlinear two-tank system. The state is

```math
x =
\begin{bmatrix}
h_1 \\
h_2
\end{bmatrix}
\in \mathbb{R}^2,
```

where `h1` and `h2` are deviations of the two liquid levels from a nominal operating point. The control input is

```math
u =
\begin{bmatrix}
u_1 \\
u_2
\end{bmatrix}
\in \mathbb{R}^2,
```

where `u1` and `u2` are controlled inflows into the two tanks.

The method class is nonlinear Lyapunov-based hybrid control. The main idea is:

1. Use a local linear controller near the origin.
2. Use an outer feedback that enforces a transport-PDE identity for a scalar function `T(x)`.
3. Switch from the outer controller to the local controller when the trajectory reaches a certified inner Lyapunov set.
4. Optionally add a swirl term tangent to the Lyapunov level sets.

## System Description

The plant is control-affine:

```math
\dot{x} = f(x) + G(x)u.
```

For this two-tank example,

```math
G(x)=I_2,
```

and

```math
f(x)=
\begin{bmatrix}
\alpha \dfrac{h_1^2}{r^2+h_1^2}h_1 + \kappa(h_2-h_1) \\
\alpha \dfrac{h_2^2}{r^2+h_2^2}h_2 + \kappa(h_1-h_2)
\end{bmatrix}.
```

Notation:

- `alpha > 0`: strength of the destabilizing nonlinear self-inflow term.
- `kappa > 0`: coupling coefficient between the two tanks.
- `r > 0`: saturation scale in the nonlinear term.
- `h1, h2`: tank-level deviations.
- `u1, u2`: control inflows.

The default implementation parameters are:

```text
alpha = 2.4
kappa = 0.8
r = 1.0
k = 1.2
sigma_level = a = 0.8
```

The generated swirl comparison artifacts use two switching levels:

```text
small switching level: a = 0.1
large switching level: a = 0.8
```

No hard input bounds are imposed in the current implementation. The simulation has an optional state norm cutoff `state_max_norm` to stop clearly invalid numerical runs.

## Mathematical Specification

The linearization at the origin is

```math
A =
\begin{bmatrix}
-\kappa & \kappa \\
\kappa & -\kappa
\end{bmatrix},
\qquad
B = I_2.
```

The local feedback is

```math
u_{\mathrm{loc}}(x)=Kx,
\qquad
K=-kI_2,\quad k>0.
```

The closed-loop linearization is

```math
A_{\mathrm{cl}}=A+BK.
```

Its eigenvalues are `-k` and `-(2 kappa + k)`, so `A_cl` is Hurwitz for every `k > 0`. The quadratic Lyapunov matrix `P` is computed from

```math
A_{\mathrm{cl}}^\top P + P A_{\mathrm{cl}} = -I.
```

For the two-tank structure this gives

```math
P =
\frac{1}{2k(2\kappa+k)}
\begin{bmatrix}
\kappa+k & \kappa \\
\kappa & \kappa+k
\end{bmatrix}.
```

The Lyapunov function used in the code is

```math
W(x)=x^\top P x.
```

The switching level is denoted by `a` in the mathematical formulas and by `sigma_level` in the code. The inner and outer regions are

```math
\Omega_a = \{x\in\mathbb{R}^2 : W(x)\le a\},
\qquad
D_a = \{x\in\mathbb{R}^2 : W(x)>a\}.
```

The switching surface is

```math
\Sigma_a = \{x\in\mathbb{R}^2 : W(x)=a\}.
```

In the outer region define

```math
T(x)=\log\frac{W(x)}{a},
\qquad
\nabla T(x)=\frac{\nabla W(x)}{W(x)}.
```

Since `G(x)=I_2`, the outer PDE controller is

```math
u_{\mathrm{ext}}(x)
=
-\frac{W(x)+\nabla W(x)^\top f(x)}
{\|\nabla W(x)\|^2}\nabla W(x),
\qquad x\in D_a.
```

This controller enforces

```math
\nabla T(x)^\top(f(x)+u_{\mathrm{ext}}(x))=-1,
```

which is equivalent to

```math
\dot{W}(x)=-W(x)
```

in the outer region. Therefore, while the trajectory stays in `D_a`,

```math
W(x(t)) = W(x_0)e^{-t}.
```

The theoretical entrance time into `Sigma_a` is

```math
\tau_{\mathrm{in}}(x_0)
=
\log\frac{W(x_0)}{a}.
```

The README gives the concise derivation needed to understand the implementation. A longer mathematical appendix can be kept in `math_appendix.pdf` if the final submission includes the full proof.

## Swirl Outer Controller

The swirl controller keeps the same Lyapunov decay law but changes the path in the phase plane.

Let

```math
g(x)=\nabla T(x),
\qquad
g^\perp(x)=
\begin{bmatrix}
-g_2(x) \\
g_1(x)
\end{bmatrix}.
```

Since `g(x)^T g^\perp(x)=0`, the tangential component does not change the transport-PDE identity. The implemented swirl control is

```math
u_{\mathrm{sw}}(x)
=
u_{\mathrm{ext}}(x)
+ \omega(x)g^\perp(x),
```

where

```math
\omega(x)
=
\beta \frac{\max(W(x)-a,0)}{W(x)+\varepsilon}.
```

Here `beta` is `swirl_gain` in code and `epsilon` is a small numerical regularization. The default run script uses `swirl_gain = 10.0`.

Because the swirl term is tangent to the level sets of `T` and `W`, it preserves

```math
\dot{W}(x)=-W(x)
```

outside the switching surface. Its purpose is visualization and trajectory shaping, not changing the certified radial Lyapunov decrease.

## Hybrid Control Law

For both the regular and swirl variants, the hybrid controller applies

```math
u(x)=
\begin{cases}
u_{\mathrm{outer}}(x), & W(x)>a,\\
u_{\mathrm{loc}}(x), & W(x)\le a,
\end{cases}
```

where `u_outer` is either `u_ext` or `u_sw`.

The explicit regular hybrid controller is

```math
u_{\mathrm{hyb}}(x)=
\begin{cases}
-\dfrac{W(x)+\nabla W(x)^\top f(x)}
{\|\nabla W(x)\|^2}\nabla W(x),
& W(x)>a,\\[3mm]
Kx,
& W(x)\le a.
\end{cases}
```

The explicit swirl hybrid controller is

```math
u_{\mathrm{hyb}}^{\mathrm{sw}}(x)=
\begin{cases}
-\dfrac{W(x)+\nabla W(x)^\top f(x)}
{\|\nabla W(x)\|^2}\nabla W(x)
+\omega(x)
\begin{bmatrix}
-\partial_{h_2}T(x)\\
\partial_{h_1}T(x)
\end{bmatrix},
& W(x)>a,\\[5mm]
Kx,
& W(x)\le a,
\end{cases}
```

with

```math
\omega(x)=\beta\frac{\max(W(x)-a,0)}{W(x)+\varepsilon}.
```

Here `beta` is `swirl_gain`, `epsilon` is the small numerical regularization, and `K=-kI_2`.

The theory assumes that the chosen level `a` is small enough for the local Lyapunov decrease condition to hold in `Omega_a`. The code uses the default value `a = 0.8`; this value should be checked numerically or justified in the final report if it is used for submission.

## Algorithm Listing

For one simulation run:

1. Build the two-tank plant with `alpha`, `kappa`, and `r`.
2. Linearize the plant at `x = 0` to obtain `A` and `B`.
3. Choose `K = -kI_2`.
4. Compute `A_cl = A + BK` and verify that it is Hurwitz.
5. Solve `A_cl^T P + P A_cl = -I`.
6. Define `W(x)=x^T P x` and the switching level `a`.
7. Build the local controller `u_loc(x)=Kx`.
8. Build either the regular outer controller `u_ext` or the swirl outer controller `u_sw`.
9. At each integration step:
   - compute `W(x)`;
   - if `W(x)>a`, apply the selected outer controller;
   - otherwise apply `u_loc`.
10. Integrate the closed-loop dynamics with fixed-step RK4.
11. Store time, state, control, Lyapunov value, and active mode.
12. Generate plots and an animation from the stored result.

## Experimental Setup

Default simulation settings are stored in `configs/default.json`. The root scripts load this file through `src/experiments/config.py`; if the file is absent, the same defaults are used from code.

```text
initial state x0 = [5.0, 5.0]^T
simulation interval = [0, 10] s
time step dt = 0.01 s
state_max_norm = 20.0
regularization_eps = 1e-12
swirl_gain = 10.0 in the swirl run script
```

The target state is the equilibrium `x = 0`. There is no reference trajectory and no parameter adaptation in the current version.

## Reproducibility

Install dependencies:

```bash
pip install -r requirements.txt
```

Run the regular hybrid controller:

```bash
python3 two_tank_hybrid.py
```

Run the hybrid controller with swirl:

```bash
python3 two_tank_swirl.py
```

With `controller.sigma_level = 0.8`, the swirl script writes to `swirl_control_big_a`. To reproduce the small switching-level case, set `controller.sigma_level = 0.1` in `configs/default.json` and run the same command; the outputs are written to `swirl_control_small_a`.

Expected generated outputs:

```text
figures/hybrid_control/plots.png
animations/hybrid_control/two_tank.gif
figures/swirl_control_big_a/plots.png
animations/swirl_control_big_a/two_tank.gif
```

The already generated swirl comparison artifacts are stored in:

```text
figures/swirl_control_small_a/plots.png
animations/swirl_control_small_a/two_tank.gif
figures/swirl_control_big_a/plots.png
animations/swirl_control_big_a/two_tank.gif
```

## Results Summary

The final results demonstrate the following claims:

1. The regular PDE hybrid controller drives the trajectory from a large initial condition into `Omega_a` and then to the origin.
2. During the outer phase, `W(x(t))` follows the theoretical exponential curve `W(x_0)e^{-t}` up to numerical integration error.
3. The swirl controller changes the visible phase-plane path while preserving the same Lyapunov decay law.
4. The swirl comparison uses the same swirl controller with two switching levels: `a=0.1` and `a=0.8`.

<p align="center">
  <img src="figures/hybrid_control/plots.png" alt="regular hybrid controller plots" width="700">
</p>
<p align="center">
  <em>Regular PDE hybrid controller: state trajectories, control inputs, phase portrait, Lyapunov decay, and switching mode.</em>
</p>

<p align="center">
  <img src="figures/swirl_control_small_a/plots.png" alt="swirl hybrid controller plots with small switching level" width="700">
</p>
<p align="center">
  <em>Swirl PDE hybrid controller with small switching level `a=0.1`: the outer swirling motion remains active longer before switching to the local stabilizer.</em>
</p>

<p align="center">
  <img src="figures/swirl_control_big_a/plots.png" alt="swirl hybrid controller plots with large switching level" width="700">
</p>
<p align="center">
  <em>Swirl PDE hybrid controller with larger switching level `a=0.8`: the trajectory enters the local region earlier.</em>
</p>

<p align="center">
  <img src="animations/swirl_control_small_a/two_tank.gif" alt="two-tank swirl hybrid animation with small switching level" width="700">
</p>
<p align="center">
  <em>Animation for `a=0.1`: the switching boundary is smaller, so the outer swirl phase persists longer.</em>
</p>

<p align="center">
  <img src="animations/swirl_control_big_a/two_tank.gif" alt="two-tank swirl hybrid animation with large switching level" width="700">
</p>
<p align="center">
  <em>Animation for `a=0.8`: the switching boundary is larger, so the local stabilizer takes over earlier.</em>
</p>

Animation note: the state variables `h1` and `h2` are level deviations. For visualization, they are converted to absolute tank volumes by adding a target volume. This target volume is chosen dynamically so that all displayed volumes remain at least 1 liter above zero, and the tank-volume axis is scaled from the minimum displayed volume minus 1 liter to the maximum displayed volume plus 1 liter.

## Current Limitations

1. The code uses a fixed-step RK4 integrator rather than an adaptive ODE solver.
2. The implementation does not impose actuator saturation.
3. The theoretical local condition requires the selected `a` to be inside a valid local Lyapunov region; the default value should be validated for the final submission.
4. The swirl term is implemented only for two-dimensional state spaces.
5. The current comparison is between two switching levels of the same swirl controller. For Projects 2 and higher, the course rules may require a stronger baseline comparison if this is not accepted as a weaker/stronger version of the same method.

## Code Structure

```text
.
├── README.md
├── requirements.txt
├── configs/
│   └── default.json
├── figures/
│   ├── hybrid_control/
│   ├── swirl_control_small_a/
│   └── swirl_control_big_a/
├── animations/
│   ├── idea.gif
│   ├── hybrid_control/
│   ├── swirl_control_small_a/
│   └── swirl_control_big_a/
├── two_tank_hybrid.py
├── two_tank_swirl.py
└── src/
    ├── cli.py
    ├── core/
    │   ├── interfaces.py
    │   └── types.py
    ├── systems/
    │   └── two_tank.py
    ├── lyapunov/
    │   └── quadratic.py
    ├── controllers/
    │   ├── local.py
    │   ├── outer.py
    │   ├── swirl.py
    │   └── hybrid.py
    ├── simulation/
    │   ├── rk4.py
    │   └── simulator.py
    ├── experiments/
    │   ├── config.py
    │   └── two_tank_setup.py
    └── visualization/
        └── two_tank.py
```

Module responsibilities:

- `core/`: abstract interfaces and shared array typing helpers.
- `systems/`: the two-tank plant and its linearization.
- `lyapunov/`: construction of the quadratic Lyapunov matrix and `W(x)`.
- `controllers/`: local, PDE outer, swirl outer, and hybrid switching controllers.
- `simulation/`: RK4 integration and closed-loop simulation result containers.
- `experiments/`: default parameters and setup factories.
- `visualization/`: plots, phase portraits, mode plots, and GIF animation helpers.
- root scripts: reproducible command-line entry points.
- `configs/default.json`: default experiment parameters recorded in a course-template-friendly format.
- `figures/`: generated static plots.
- `animations/`: generated GIF animations.

The system model is separated from the controller logic, simulation is separated from plotting, and visualization does not contain core dynamics or control laws.

## Repository Notes for Submission

The repository now follows the required folder template: it has `README.md`, `src/`, `figures/`, `animations/`, and `configs/`. Before final submission, regenerate the plots and GIFs so that the placeholder directories contain the current outputs.

The project folder name also needs to follow the course pattern:

```text
project_<number>_<topic_name>_<system_name>
```

Rename the outer project directory accordingly before submission if this repository is submitted as a course project folder.
