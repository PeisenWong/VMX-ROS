import pandas as pd
import matplotlib.pyplot as plt

# Load the logged data
df = pd.read_csv("/home/pi/rpm_log.csv")

# Plot all 3 wheels
plt.figure(figsize=(12, 8))

# LEFT Wheel
plt.plot(df["timestamp"], df["left_target"], linestyle='--', label="Target RPM - Left")
plt.plot(df["timestamp"], df["left_measured"], linestyle='-', label="Measured RPM - Left")

# RIGHT Wheel
plt.plot(df["timestamp"], df["right_target"], linestyle='--', label="Target RPM - Right")
plt.plot(df["timestamp"], df["right_measured"], linestyle='-', label="Measured RPM - Right")

# BACK Wheel
plt.plot(df["timestamp"], df["back_target"], linestyle='--', label="Target RPM - Back")
plt.plot(df["timestamp"], df["back_measured"], linestyle='-', label="Measured RPM - Back")

# Labels and aesthetics
plt.title("Target vs Measured RPM (PID Tracking)")
plt.xlabel("Time (s)")
plt.ylabel("RPM")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
