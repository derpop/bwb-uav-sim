"""Plot the torque-free tumble demo written by `build/tumble`.

Usage:
    ./build/tumble > out/tumble.csv
    python3 analysis/plot_tumble.py out/tumble.csv   # writes out/tumble.png
"""

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]  # fixed order: x/roll, y/pitch, z/yaw
INK, MUTED, GRID = "#1f1f1e", "#6b6a64", "#e4e3dc"


def main(csv_path: str) -> None:
    d = np.genfromtxt(csv_path, delimiter=",", names=True)
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True, constrained_layout=True)

    panels = [
        ("Body rates (rad/s)", ["p", "q", "r"], 1.0),
        ("Euler angles (deg)", ["phi", "theta", "psi"], 180 / np.pi),
    ]
    for ax, (title, cols, scale) in zip(axes, panels):
        for col, color in zip(cols, SERIES):
            ax.plot(d["t"], d[col] * scale, color=color, lw=2, label=col)
        ax.set_title(title, loc="left", color=INK, fontsize=11)
        ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=3, frameon=False)

    ax = axes[2]
    ax.plot(d["t"], np.abs(d["dH_rel"]), color=SERIES[0], lw=2, label="|H| drift")
    ax.plot(d["t"], np.abs(d["dKE_rel"]), color=SERIES[1], lw=2, label="KE drift")
    ax.set_yscale("log")
    ax.set_ylim(1e-17, 1e-12)
    ax.set_title("Invariant drift (relative; round-off level)", loc="left", color=INK, fontsize=11)
    ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=2, frameon=False)
    ax.set_xlabel("time (s)", color=MUTED)

    for a in axes:
        a.grid(color=GRID, lw=0.8)
        a.tick_params(colors=MUTED)
        for s in a.spines.values():
            s.set_visible(False)

    fig.suptitle("Aerosonde torque-free tumble about the intermediate (pitch) axis", color=INK)
    out = Path(csv_path).with_suffix(".png")
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out/tumble.csv")
