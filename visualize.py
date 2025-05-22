import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load the logged data
df = pd.read_csv("/home/pi/rpm_log.csv")

# Wheels configuration
wheels = {
    'Left': ('left_target', 'left_measured'),
    'Right': ('right_target', 'right_measured'),
    'Back': ('back_target', 'back_measured')
}

def step_metrics(time, measured, target):
    # assume step happens at first nonzero target
    t0_idx = np.argmax(target > 0)
    t0 = time[t0_idx]
    tgt = target[t0_idx]
    mask = time >= t0
    t = time[mask] - t0
    y = measured[mask]
    # Rise time 10%→90%
    y10, y90 = 0.1 * tgt, 0.9 * tgt
    try:
        t1 = t[np.where(y >= y10)[0][0]]
        t2 = t[np.where(y >= y90)[0][0]]
        rise = t2 - t1
    except IndexError:
        rise = np.nan
    # Overshoot
    peak = y.max()
    overshoot = (peak - tgt) / tgt * 100 if tgt != 0 else np.nan
    # Settling time (first time within ±2% and stays)
    tol = 0.02 * tgt
    settled = np.where(np.abs(y - tgt) <= tol)[0]
    settle = (t[settled[0]] if len(settled) else np.nan) if tgt != 0 else np.nan
    return rise, overshoot, settle

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 12))

for ax, (name, (col_t, col_m)) in zip(axes, wheels.items()):
    time = df["timestamp"].values
    target = df[col_t].values
    meas   = df[col_m].values

    # plot
    l1, = ax.plot(time, target, '--', label="Target RPM")
    l2, = ax.plot(time, meas,   '-',  label="Measured RPM")

    ax.set_title(f"{name} Wheel")
    ax.set_ylabel("RPM")
    ax.grid(True)

    # compute & annotate metrics
    rise, os_, settle = step_metrics(time, meas, target)
    txt = f"Rise Time: {rise:.2f}s\n%OS: {os_:.1f}%\nSettle Time: {settle:.2f}s"
    ax.text(0.95, 0.10, txt,
            transform=ax.transAxes,
            ha='right', va='bottom',
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7))

axes[-1].set_xlabel("Time (s)")

# shared legend at top
fig.legend(handles=[l1, l2],
           labels=["Target RPM", "Measured RPM"],
           loc='upper center',
           ncol=2,
           frameon=False)

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()
