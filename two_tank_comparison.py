from common import X0, create_tank_animation, make_output_dir, save_comparison_plots, simulate_all, valid_mask


def main():
    output_dir = make_output_dir("comparison")
    local_data, hybrid_data = simulate_all(X0)

    save_comparison_plots(local_data, hybrid_data, X0, output_dir)

    t_hyb, x_hyb, _, _, region_hyb, _ = hybrid_data
    _, x_local, _, _, _, _ = local_data
    create_tank_animation(
        t=t_hyb,
        xs=x_hyb,
        region=region_hyb,
        output_path=output_dir / "two_tank.gif",
        title="Two-Tank Comparison",
        mode_label="Hybrid",
        comparison_xs=x_local,
        comparison_valid=valid_mask(x_local),
    )

    print(f"Plots saved to {output_dir / 'plots.png'}")
    print(f"Animation saved to {output_dir / 'two_tank.gif'}")


if __name__ == "__main__":
    main()
