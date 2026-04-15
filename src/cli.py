from __future__ import annotations

from experiments.config import load_experiment_config
from experiments.two_tank_setup import run_two_tank_experiment
from visualization import (
    create_tank_animation,
    make_animation_dir,
    make_figure_dir,
    print_summary,
    save_hybrid_plots,
)


def _print_linear_design(setup) -> None:
    print("\nA_cl =")
    print(setup.lyapunov_data.Acl)
    print("\nP =")
    print(setup.lyapunov_data.P)
    print("\nEigenvalues(A_cl) =", setup.lyapunov_data.eig_Acl)
    print("Eigenvalues(P)    =", setup.lyapunov_data.eig_P)


def hybrid_main() -> None:
    config = load_experiment_config()
    figure_dir = make_figure_dir("hybrid_control")
    animation_dir = make_animation_dir("hybrid_control")
    setup, result = run_two_tank_experiment(
        x0=config.x0,
        config=config.simulation,
        alpha=config.alpha,
        kappa=config.kappa,
        r=config.r,
        k=config.k,
        sigma_level=config.sigma_level,
        use_swirl=False,
    )

    print_summary("PDE hybrid", setup, config.x0, result)
    _print_linear_design(setup)

    save_hybrid_plots(
        setup=setup,
        result=result,
        x0=config.x0,
        output_dir=figure_dir,
        figure_title="Two-Tank PDE Hybrid Control",
    )
    create_tank_animation(
        setup=setup,
        result=result,
        output_path=animation_dir / "two_tank.gif",
        title="Two-Tank PDE Hybrid Control",
        mode_label="PDE hybrid",
    )

    print(f"Plots saved to {figure_dir / 'plots.png'}")
    print(f"Animation saved to {animation_dir / 'two_tank.gif'}")


def swirl_main() -> None:
    config = load_experiment_config()
    output_name = "swirl_control_small_a" if config.sigma_level <= 0.1 else "swirl_control_big_a"
    figure_dir = make_figure_dir(output_name)
    animation_dir = make_animation_dir(output_name)
    setup, result = run_two_tank_experiment(
        x0=config.x0,
        config=config.simulation,
        alpha=config.alpha,
        kappa=config.kappa,
        r=config.r,
        k=config.k,
        sigma_level=config.sigma_level,
        use_swirl=True,
        swirl_gain=config.swirl_gain,
    )

    print_summary("PDE hybrid + swirl", setup, config.x0, result)
    _print_linear_design(setup)

    save_hybrid_plots(
        setup=setup,
        result=result,
        x0=config.x0,
        output_dir=figure_dir,
        figure_title="Two-Tank PDE Hybrid Control with Swirl",
    )
    create_tank_animation(
        setup=setup,
        result=result,
        output_path=animation_dir / "two_tank.gif",
        title="Two-Tank PDE Hybrid Control with Swirl",
        mode_label="PDE hybrid + swirl",
    )

    print(f"Plots saved to {figure_dir / 'plots.png'}")
    print(f"Animation saved to {animation_dir / 'two_tank.gif'}")
