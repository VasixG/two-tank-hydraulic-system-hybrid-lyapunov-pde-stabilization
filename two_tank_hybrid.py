from common import X0, create_tank_animation, make_output_dir, print_summary, save_hybrid_plots, simulate


def main():
    output_dir = make_output_dir("hybrid_control")
    t, xs, us, ws, region, alive = simulate(X0, "hybrid")

    print_summary("hybrid", X0, t, xs, ws, alive)
    save_hybrid_plots(t, xs, us, ws, region, X0, output_dir)
    create_tank_animation(
        t=t,
        xs=xs,
        region=region,
        output_path=output_dir / "two_tank.gif",
        title="Two-Tank Hybrid Control",
        mode_label="Hybrid",
    )

    print(f"Plots saved to {output_dir / 'plots.png'}")
    print(f"Animation saved to {output_dir / 'two_tank.gif'}")


if __name__ == "__main__":
    main()
