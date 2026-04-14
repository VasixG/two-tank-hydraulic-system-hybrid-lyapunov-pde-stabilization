from common import X0, create_tank_animation, make_output_dir, print_summary, save_local_plots, simulate


def main():
    output_dir = make_output_dir("local_control")
    t, xs, us, ws, region, alive = simulate(X0, "local")

    print_summary("local", X0, t, xs, ws, alive)
    save_local_plots(t, xs, us, ws, alive, output_dir)
    create_tank_animation(
        t=t,
        xs=xs,
        region=region,
        output_path=output_dir / "two_tank.gif",
        title="Two-Tank Local Control",
        mode_label="Local only",
    )

    print(f"Plots saved to {output_dir / 'plots.png'}")
    print(f"Animation saved to {output_dir / 'two_tank.gif'}")


if __name__ == "__main__":
    main()
