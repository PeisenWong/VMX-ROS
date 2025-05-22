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

# Metrics storage
metrics = []

def step_metrics(time, measured, target):
    # assume step happens at t0 = first nonzero target
    t0_idx = np.argmax(target > 0)
    t0 = time[t0_idx]
    tgt = target[t0_idx]
    window = (time >= t0)
    t = time[window] - t0
    y = measured[window]
    # rise time 10%→90%
    y10 = 0.1 * tgt
    y90 = 0.9 * tgt
    try:
        t_rise_start = t[np.where(y >= y10)[0][0]]
        t_rise_end   = t[np.where(y >= y90)[0][0]]
        rise_time = t_rise_end - t_rise_start
    except IndexError:
        rise_time = np.nan
    # overshoot
    peak = np.max(y)
    overshoot = (peak - tgt) / tgt * 100.0 if tgt != 0 else np.nan
    # settling time within ±2%
    tol = 0.02 * tgt
    settled_idx = np.where(np.abs(y - tgt) <= tol)[0]
    settling_time = (t[settled_idx[0]] if len(settled_idx)>0 else np.nan) if tgt!=0 else np.nan
    return rise_time, overshoot, settling_time

# Plot
fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 12))

for ax, (name, (col_t, col_m)) in zip(axes, wheels.items()):
    time = df["timestamp"].values
    target = df[col_t].values
    measured = df[col_m].values

    # compute metrics
    rise, over, settle = step_metrics(time, measured, target)
    metrics.append((name, rise, over, settle))

    ax.plot(time, target, '--', label="Target")
    ax.plot(time, measured, '-', label="Measured")
    ax.set_title(f"{name} Wheel")
    ax.set_ylabel("RPM")
    ax.grid(True)
    ax.legend()
    # annotate
    ax.text(0.95, 0.10,
            f"Rise: {rise:.2f}s\nOS: {over:.1f}%\nSettle: {settle:.2f}s",
            transform=ax.transAxes, ha='right', va='bottom',
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.7))

axes[-1].set_xlabel("Time (s)")
plt.tight_layout()
plt.show()

# print summary
print("Wheel   | Rise(s) | Overshoot(%) | Settling(s)")
print("--------+---------+--------------+-------------")
for name, rise, over, settle in metrics:
    print(f"{name:<7} | {rise:7.2f} | {over:12.1f} | {settle:11.2f}")
