import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# expects dive.csv (pitch_demo) and trimmed.csv (pitch_demo --trim)
dive = pd.read_csv("dive.csv")
trimmed = pd.read_csv("trimmed.csv")

BLUE, ORANGE = "#2a78d6", "#eb6834"
INK, INK_2, GRID = "#0b0b0b", "#52514e", "#e6e5e0"

runs = [
    ("Untrimmed (δe = 0, δt = 0.5)", dive, ORANGE),
    ("Trimmed (δe = −6.26°, δt = 0.334)", trimmed, BLUE),
]

alpha_dive_pred = np.degrees(-(-0.02338) / (-0.38))  # -C_m0 / C_m_alpha
alpha_trim = np.degrees(trimmed["alpha"].iloc[0])     # what trim() solved for

plt.rcParams.update({
    "axes.edgecolor": INK_2, "axes.labelcolor": INK, "text.color": INK,
    "xtick.color": INK_2, "ytick.color": INK_2,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "font.size": 10,
})

fig = plt.figure(figsize=(8, 10))
gs = fig.add_gridspec(3, 1, height_ratios=[1.4, 1, 0.8], hspace=0.45, top=0.90)
ax_path = fig.add_subplot(gs[0])
ax_ang = fig.add_subplot(gs[1])
ax_va = fig.add_subplot(gs[2], sharex=ax_ang)

# 1. side view: distance north vs altitude, both runs
for label, df, color in runs:
    ax_path.plot(df["north"], df["altitude"], color=color, lw=2, label=label)

# per-second dots and time labels on the dive only
whole = np.isclose(dive["t"], dive["t"].round(), atol=1e-6)
ax_path.plot(dive["north"][whole], dive["altitude"][whole], "o", color=ORANGE, ms=5,
             mec="white", mew=1.5)
for _, row in dive[whole].iterrows():
    if int(round(row["t"])) in (2, 4, 6, 8, 10):
        ax_path.annotate(f"{row['t']:.0f} s", (row["north"], row["altitude"]),
                         xytext=(8, 4), textcoords="offset points", color=INK_2)
ax_path.set_aspect("equal", adjustable="box")
ax_path.set_xlim(-10, dive["north"].max() + 50)
ax_path.set_xlabel("Distance north (m)")
ax_path.set_ylabel("Altitude (m)")
ax_path.set_title("Trajectory, side view (10 s)", loc="left")

# 2. alpha: each run settles where its moment balance says
for label, df, color in runs:
    ax_ang.plot(df["t"], np.degrees(df["alpha"]), color=color, lw=2)
ax_ang.axhline(alpha_dive_pred, color=ORANGE, lw=1, ls="--")
ax_ang.axhline(alpha_trim, color=BLUE, lw=1, ls="--")
ax_ang.annotate(f"$-C_{{m0}}/C_{{m\\alpha}}$ = {alpha_dive_pred:.1f}° (δe = 0)",
                (dive["t"].iloc[-1], alpha_dive_pred), xytext=(0, -14),
                textcoords="offset points", ha="right", color=INK_2)
ax_ang.annotate(f"trim α = {alpha_trim:.2f}°",
                (trimmed["t"].iloc[-1], alpha_trim), xytext=(0, 6),
                textcoords="offset points", ha="right", color=INK_2)
ax_ang.set_ylabel("Angle of attack α (deg)")
ax_ang.set_title("Each run settles at the α its moment balance predicts", loc="left")

# 3. airspeed
for label, df, color in runs:
    ax_va.plot(df["t"], df["Va"], color=color, lw=2)
ax_va.set_ylabel("Airspeed (m/s)")
ax_va.set_xlabel("Time (s)")
ax_va.set_title("Airspeed", loc="left")
plt.setp(ax_ang.get_xticklabels(), visible=False)

fig.legend(*ax_path.get_legend_handles_labels(), loc="upper left",
           bbox_to_anchor=(0.06, 0.955), ncol=2, frameon=False)
fig.suptitle("Aerosonde from level flight at 25 m/s: untrimmed vs trimmed",
             x=0.06, y=0.985, ha="left", fontsize=11)
fig.savefig("docs/img/trim_vs_dive.png", dpi=150, bbox_inches="tight", facecolor="white")
