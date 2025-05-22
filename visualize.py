import pandas as pd
import matplotlib.pyplot as plt

# Load the logged data
df = pd.read_csv("/home/pi/rpm_log.csv")

# Wheels configuration
wheels = {
    'Left':   ('left_target',  'left_measured'),
    'Right':  ('right_target', 'right_measured'),
    'Back':   ('back_target',  'back_measured')
}

# Performance metric functions
def rise_time(time, signal, low=0.1, high=0.9):
    base = signal.iloc[0]
    final = signal.iloc[-1]
    low_lv  = base + low  * (final - base)
    high_lv = base + high * (final - base)
    t_low  = time[signal >= low_lv].iloc[0]
    t_high = time[signal >= high_lv].iloc[0]
    return t_high - t_low

def overshoot(signal, target):
    peak = signal.max()
    return (peak - target) / target * 100

def settling_time(time, signal, target, tol=0.02):
    hi = target * (1 + tol)
    lo = target * (1 - tol)
    outside = (signal < lo) | (signal > hi)
    if outside.any():
        last_out = time[outside].iloc[-1]
        return time.iloc[-1] - last_out
    else:
        return 0.0

# Create plots
fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 12))

for ax, (name, (tcol, mcol)) in zip(axes, wheels.items()):
    t = df["timestamp"]
    yt = df[tcol]
    ym = df[mcol]
    ax.plot(t, yt, '--', label=f"Target RPM")
    ax.plot(t, ym, '-' , label=f"Measured RPM")
    ax.set_title(f"{name} Wheel")
    ax.set_ylabel("RPM")
    ax.grid(True)
    ax.legend(loc="upper left")

    # Compute metrics
    rt  = rise_time(t, ym)
    os_ = overshoot(ym, yt.iloc[-1])
    st  = settling_time(t, ym, yt.iloc[-1])

    # Annotate
    txt = (f"Rise time: {rt:.2f}s\n"
           f"Overshoot: {os_:.1f}%\n"
           f"Settling time: {st:.2f}s")
    ax.text(0.05, 0.75, txt, transform=ax.transAxes,
            bbox=dict(facecolor='white', alpha=0.7))

axes[-1].set_xlabel("Time (s)")
plt.tight_layout()
plt.show()
