import pandas as pd
import matplotlib.pyplot as plt

# Load the logged data
df = pd.read_csv("rpm_log.csv")

# Plot all 3 wheels
plt.figure(figsize=(12, 8))

# LEFT Wheel
plt.plot(df["time"], df["target_left"], linestyle='--', label="Target RPM - Left")
plt.plot(df["time"], df["measured_left"], linestyle='-', label="Measured RPM - Left")

# RIGHT Wheel
plt.plot(df["time"], df["target_right"], linestyle='--', label="Target RPM - Right")
plt.plot(df["time"], df["measured_right"], linestyle='-', label="Measured RPM - Right")

# BACK Wheel
plt.plot(df["time"], df["target_back"], linestyle='--', label="Target RPM - Back")
plt.plot(df["time"], df["measured_back"], linestyle='-', label="Measured RPM - Back")

# Labels and aesthetics
plt.title("Target vs Measured RPM (PID Tracking)")
plt.xlabel("Time (s)")
plt.ylabel("RPM")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
