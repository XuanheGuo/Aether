"""Plot the archived native states or regenerate plots from their CSV files."""

import argparse
import csv
from pathlib import Path

import numpy as np


def plot(output, histories):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt

    def signal(rows, key):
        return np.array([float(row[key]) if row[key] not in (None, "") else np.nan for row in rows])

    vertical = histories["vertical"]
    time = signal(vertical, "time_s")
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True, layout="constrained")
    for ax, key, ref, label in zip(axes[:3],
                                 ("com_position_m_z", "com_velocity_m_s_z", "com_accel_velocity_fd_m_s2_z"),
                                 ("reference_com_z_m", "reference_com_vz_m_s", "reference_com_az_m_s2"),
                                 ("COM z (m)", "COM vz (m/s)", "COM az (m/s²)"), strict=True):
        ax.plot(time, signal(vertical, key), label="MuJoCo")
        ax.plot(time, signal(vertical, ref), "--", label="Contact-free reference")
        ax.set_ylabel(label)
    axes[2].set_ylim(-15, 10)
    axes[2].set_title("Impact peak clipped here; full values in CSV", fontsize=9)
    for field, label in (("applied_test", "Test"), ("gravity", "Gravity"),
                         ("contact", "Contact"), ("total_external", "Total external")):
        axes[3].plot(time, signal(vertical, f"{field}_force_N_z"), label=label)
    axes[3].set_ylim(-125, 175)
    axes[3].set_title("Impact peak clipped here; full values in CSV", fontsize=9)
    axes[3].set_ylabel("World Z force (N)")
    contact = signal(vertical, "contact_points") > 0
    for ax in axes:
        ax.fill_between(time, 0, 1, where=contact, alpha=.09, color="gray",
                        transform=ax.get_xaxis_transform(), label="Contact")
        ax.grid(alpha=.25)
        ax.legend(loc="upper left", ncols=3, fontsize=8)
    axes[-1].set_xlabel("Simulation time (s)")
    fig.suptitle("Vehicle-V1 · Vertical load, departure and recontact")
    fig.savefig(output / "vertical.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(3, 2, figsize=(11, 8), sharex=True, layout="constrained")
    for name, color in (("axis", "#238588"), ("tumble", "#4656ad"), ("inertia_variant", "#e08424")):
        rows = histories[name]
        time = signal(rows, "time_s")
        for i, axis in enumerate("xyz"):
            axes.flat[i].plot(time, signal(rows, f"omega_body_rad_s_{axis}"), label=name, color=color)
            axes.flat[i].set_ylabel(f"Body ω{axis} (rad/s)")
        for ax, key, label in zip((axes[1, 1], axes[2, 0], axes[2, 1]),
                                  ("body_quat_w", "angular_momentum_relative_drift", "kinetic_energy_relative_drift"),
                                  ("Quaternion w", "Relative angular momentum drift", "Relative energy drift"),
                                  strict=True):
            ax.plot(time, signal(rows, key), label=name, color=color)
            ax.set_ylabel(label)
    for ax in axes.flat:
        ax.grid(alpha=.25)
    for ax in axes[-1]:
        ax.set_xlabel("Simulation time (s)")
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Vehicle-V1 · Torque-free initial states and inertia sensitivity")
    fig.savefig(output / "attitude.png", dpi=150)
    plt.close(fig)


def main():
    from prototypes.aether_v1.motion.experiment import CASES

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    histories = {}
    for case in CASES:
        with (args.run / f"{case}.csv").open() as stream:
            histories[case] = list(csv.DictReader(stream))
    plot(args.run, histories)


if __name__ == "__main__":
    main()
