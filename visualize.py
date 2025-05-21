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

# Plot each wheel in its own figure
for name, (col_target, col_measured) in wheels.items():
    plt.figure(figsize=(10, 6))
    plt.plot(df["timestamp"], df[col_target], linestyle='--', label=f"Target RPM - {name}")
    plt.plot(df["timestamp"], df[col_measured], linestyle='-', label=f"Measured RPM - {name}")
    plt.title(f"{name} Wheel: Target vs Measured RPM")
    plt.xlabel("Time (s)")
    plt.ylabel("RPM")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
