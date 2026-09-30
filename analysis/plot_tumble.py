import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os

df = pd.read_csv("tumble.csv")

fig, axes = plt.subplots(3, 1, figsize=(8, 8))

ax = axes[0]
ax.plot(df['t'], df['p'], label='p')
ax.plot(df['t'], df['q'], label='q')
ax.plot(df['t'], df['r'], label='r')
ax.set_title('rate (rad/s)')
ax.legend()
ax.grid(alpha = 0.3)

ax = axes[1]
ax.plot(df['t'], np.degrees(df['phi']), label='phi')
ax.plot(df['t'], np.degrees(df['theta']), label='theta')
ax.plot(df['t'], np.degrees(df['psi']), label='psi')
ax.set_title('Attitude (deg)')
ax.legend()
ax.grid(alpha = 0.3)

ax = axes[2]
ax.plot(df['t'], df['T_drift'], label='Delta T')
ax.set_title('Relative energy drift')
ax.grid(alpha = 0.3)

axes[-1].set_xlabel('Time (s)')
fig.suptitle("Aerosonde intermediate-axis spin (torque-free)")
fig.tight_layout()
os.makedirs("docs/img", exist_ok=True)
fig.savefig("docs/img/tumble.png", dpi=150)