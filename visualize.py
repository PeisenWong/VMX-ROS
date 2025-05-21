import pandas as pd
import matplotlib.pyplot as plt

# Load the logged data
df = pd.read_csv("/home/pi/rpm_log.csv")

# Wheels configuration
wheels = {
    'Left': ('left_target', 'left_measured'),
    'Right': ('right_target', 'right_measured'),
    'Back': ('back_target', 'back_measured')
}

# Create a single figure with 3 vertical subplots sharing the x-axis
fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 12))

for ax, (name, (col_target, col_measured)) in zip(axes, wheels.items()):
    ax.plot(df["timestamp"], df[col_target], linestyle='--', label=f"Target RPM - {name}")
    ax.plot(df["timestamp"], df[col_measured], linestyle='-', label=f"Measured RPM - {name}")
    ax.set_title(f"{name} Wheel: Target vs Measured RPM")
    ax.set_ylabel("RPM")
    ax.grid(True)
    ax.legend()

# Common X label at bottom
axes[-1].set_xlabel("Time (s)")

plt.tight_layout()
plt.show()
