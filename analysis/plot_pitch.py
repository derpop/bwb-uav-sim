import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


df = pd.read_csv("pitch.csv")

alpha = np.degrees(df["alpha"])
gamma = np.degrees(df["theta"] - df["alpha"])  # flight path angle
alpha_pred = np.degrees(-(-0.02338) / (-0.38))  # -C_m0 / C_m_alpha

BLUE, ORANGE = "#2a78d6", "#eb6834"
INK, INK_2, GRID = "#0b0b0b", "#52514e", "#e6e5e0"

plt.rcParams.update({
    "axes.edgecolor": INK_2, "axes.labelcolor": INK, "text.color": INK,
    "xtick.color": INK_2, "ytick.color": INK_2,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "font.size": 10,
})

fig = plt.figure(figsize=(8, 10))
gs = fig.add_gridspec(3, 1, height_ratios=[1.4, 1, 0.8], hspace=0.45, top=0.92)
ax_path = fig.add_subplot(gs[0])
ax_ang = fig.add_subplot(gs[1])
ax_va = fig.add_subplot(gs[2], sharex=ax_ang)

# 1. side view of the trajectory, a dot every second
ax_path.plot(df["north"], df["altitude"], color=BLUE, lw=2)
whole = np.isclose(df["t"], df["t"].round(), atol=1e-6)
ax_path.plot(df["north"][whole], df["altitude"][whole], "o", color=BLUE, ms=5,
             mec="white", mew=1.5)
for _, row in df[whole].iterrows():
    if int(round(row["t"])) in (0, 2, 4, 6, 8, 10):
        ax_path.annotate(f"{row['t']:.0f} s", (row["north"], row["altitude"]),
                         xytext=(8, 4), textcoords="offset points", color=INK_2)
ax_path.set_aspect("equal", adjustable="box")
ax_path.set_xlim(-10, df["north"].max() + 50)
ax_path.set_xlabel("Distance north (m)")
ax_path.set_ylabel("Altitude (m)")
ax_path.set_title("Trajectory, side view", loc="left")

# 2. alpha settles fast, the path keeps steepening
ax_ang.axhline(alpha_pred, color=INK_2, lw=1, ls="--")
ax_ang.annotate(f"hand prediction, $-C_{{m0}}/C_{{m\\alpha}}$ = {alpha_pred:.1f}°",
                (df["t"].iloc[-1], alpha_pred), xytext=(0, 6), textcoords="offset points",
                ha="right", color=INK_2)
ax_ang.plot(df["t"], alpha, color=BLUE, lw=2, label="α, angle of attack")
ax_ang.plot(df["t"], gamma, color=ORANGE, lw=2, label="γ, flight path angle")
ax_ang.annotate("α", (df["t"].iloc[-1], alpha.iloc[-1]), xytext=(6, -12),
                textcoords="offset points", color=INK)
ax_ang.annotate("γ", (df["t"].iloc[-1], gamma.iloc[-1]), xytext=(6, -4),
                textcoords="offset points", color=INK)
ax_ang.set_ylabel("Angle (deg)")
ax_ang.legend(loc="lower left", frameon=False)
ax_ang.set_title("α settles in about a second; the path keeps bending down", loc="left")

# 3. airspeed
ax_va.plot(df["t"], df["Va"], color=BLUE, lw=2)
ax_va.set_ylabel("Airspeed (m/s)")
ax_va.set_xlabel("Time (s)")
ax_va.set_title("Airspeed", loc="left")
plt.setp(ax_ang.get_xticklabels(), visible=False)

fig.suptitle("Aerosonde, untrimmed (δe = 0, δt = 0.5), from level flight at 25 m/s",
             x=0.06, ha="left", fontsize=11)
fig.savefig("docs/img/dive.png", dpi=150, bbox_inches="tight", facecolor="white")
