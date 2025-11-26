import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

# === 1. Load your logged CSV ===
csv_path = "/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Logs/SizeRenderLog_20251031_163430.csv"
csv_path = "/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Logs/SizeRenderLog_20251031_163430.csv"
df = pd.read_csv(csv_path)

time = df["time"].values
halfW01 = df["halfW01"].values
duties = df[["d0", "d1", "d2", "d3"]].values

# === 2. Static plots ===
plt.figure(figsize=(8, 4))
plt.plot(time, halfW01, color='teal', linewidth=2)
plt.title("Normalized Size (halfW01) vs Time")
plt.xlabel("Time [s]")
plt.ylabel("halfW01")
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(8, 5))
for i, col in enumerate(["d0", "d1", "d2", "d3"]):
    plt.plot(time, duties[:, i], label=col)
plt.title("Duty Intensities vs Time")
plt.xlabel("Time [s]")
plt.ylabel("Duty Intensity (0–14)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure(figsize=(8, 5))
for i, col in enumerate(["d0", "d1", "d2", "d3"]):
    plt.scatter(halfW01, duties[:, i], s=10, alpha=0.6, label=col)
plt.title("Duty vs Normalized Size")
plt.xlabel("halfW01")
plt.ylabel("Duty Intensity (0–14)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# === 3. Interactive plot: slider = halfW01 ===
fig, ax = plt.subplots(figsize=(6, 4))
plt.subplots_adjust(bottom=0.25)

x_labels = ["d0", "d1", "d2", "d3"]
x = np.arange(4)
bars = ax.bar(x, duties[0], color="royalblue")
ax.set_xticks(x)
ax.set_xticklabels(x_labels)
ax.set_ylim(0, 14)
ax.set_ylabel("Duty Intensity")
ax.set_title(f"Duty per Actuator at halfW01={halfW01[0]:.3f} (t={time[0]:.2f}s)")

# === Slider: ranges over halfW01 values ===
ax_slider = plt.axes([0.15, 0.1, 0.7, 0.03])
slider = Slider(ax_slider, "halfW01", np.min(halfW01), np.max(halfW01),
                valinit=halfW01[0], valfmt="%.3f")

# === Update function ===
def update(val):
    target = slider.val
    # find closest frame to this halfW01 value
    idx = (np.abs(halfW01 - target)).argmin()
    for i, b in enumerate(bars):
        b.set_height(duties[idx, i])
    ax.set_title(f"Duty per Actuator at halfW01={halfW01[idx]:.3f} (t={time[idx]:.2f}s)")
    fig.canvas.draw_idle()

slider.on_changed(update)

plt.show()
