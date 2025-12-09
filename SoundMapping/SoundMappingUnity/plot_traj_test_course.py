import json
import numpy as np
import matplotlib.pyplot as plt

# ------------------------------
# Load JSON files
# ------------------------------
with open("Assets/Data/default/Trajectories/Setup_H_NO_20251209_195104_traj.json") as f:
    traj = json.load(f)

with open("Assets/Data/default/Trajectories/stars.json") as f:
    stars = json.load(f)

# ------------------------------
# Extract trial window ("Run")
# ------------------------------
trial = traj["trials"][0]
t0 = trial["startGameTime"]
t1 = trial["endGameTime"]

print(f"Fenêtre 'Run' (temps de jeu): start={t0:.3f}s, stop={t1:.3f}s, total={t1 - t0:.3f}s")

# ------------------------------
# Extract drone trajectories
# ------------------------------
drones = traj["trajectories"]

# Build aligned arrays of positions during the Run
centroid_positions = []
all_positions = []   # list of all agents per frame (for inter-agent distances)

for drone in drones:
    frames = drone["frames"]
    
    xs, ys, zs, ts = [], [], [], []
    for f in frames:
        if t0 <= f["t"] <= t1:
            xs.append(f["x"])
            ys.append(f["y"])
            zs.append(f["z"])
            ts.append(f["t"])

    all_positions.append(np.vstack([xs, ys, zs]).T)

# Ensure all drones have same number of samples by trimming to min
min_len = min(pos.shape[0] for pos in all_positions)
all_positions = [pos[:min_len] for pos in all_positions]

# ------------------------------
# Centroid distance to embodied drone
# ------------------------------
embodied_name = traj["embodiedName"]

embodied_traj = None
for drone in drones:
    if drone["name"] == embodied_name:
        embodied_traj = drone
        break

emb_frames = [
    f for f in embodied_traj["frames"] if t0 <= f["t"] <= t1
]
emb_positions = np.array([[f["x"], f["y"], f["z"]] for f in emb_frames])[:min_len]

stack = np.stack(all_positions)  # shape: (N_drones, N_frames, 3)
centroid = np.mean(stack, axis=0)

dist_centroid = np.linalg.norm(centroid - emb_positions, axis=1)
dist_centroid_mean = dist_centroid.mean()

print(f"Distance Centroid→Référence (Run): {dist_centroid_mean:.3f} m")
print(f"  (total {min_len} échantillons, 0 exclus)")

# ------------------------------
# Inter-agent distances
# ------------------------------
mean_dists = []
for i in range(len(all_positions)):
    for j in range(i+1, len(all_positions)):
        dij = np.linalg.norm(all_positions[i] - all_positions[j], axis=1)
        mean_dists.append(dij.mean())

mean_inter = np.mean(mean_dists)

print(f"Distance Inter-agent (Swarm Complet): {mean_inter:.3f} m")

# ------------------------------
# Stars collected (from stars.json)
# ------------------------------
# Expected format:
# [
#   {"gap": 0, "time": 4.21, "drone": "Drone3"},
#   ...
# ]

gap_counts = {}
for ev in stars:
    g = ev["gap"]
    gap_counts[g] = gap_counts.get(g, 0) + 1

print("\nEtoiles collectées:")
for g in sorted(gap_counts):
    print(f"Gap {g}: {gap_counts[g]}")

# ------------------------------
# Simple plot of trajectories
# ------------------------------
plt.figure(figsize=(6,6))
for drone, pos in zip(drones, all_positions):
    plt.plot(pos[:,0], pos[:,1], label=drone["name"])
    
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Trajectoires des drones")
plt.legend()
plt.axis("equal")
plt.show()
