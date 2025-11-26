# Re-run the plotting with the most recent JSON, same as above.
import json, math, glob, os
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

REF_SCALE = 0.3
REF_STEPS = [
    (0, 140), (-140, 0), (0, 100), (100, 0),
    (0, 160), (-100, 0), (0, 100), (-140, 0),
    (0, -160), (-200, 0), (0, 100), (100, 0),
]
OUT_TRAJ_PNG = "outputs/one_script_trajectories.png"
OUT_ERR_PNG  = "outputs/one_script_centroid_error.png"

candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Scene_Selector_H_NO_20251026_132101_traj.json"), key=os.path.getmtime, reverse=True)
if not candidates:
    raise FileNotFoundError("No JSON files found in /mnt/data. Please upload the recorded JSON.")
INPUT_JSON = Path(candidates[0])

with INPUT_JSON.open("r") as f:
    data = json.load(f)

scene = data.get("scene", data.get("level", "Unknown Scene"))

drone_tracks = {}
if "trajectories" in data:
    for i, traj in enumerate(data["trajectories"]):
        name = traj.get("name", f"id:{traj.get('id', i)}")
        frames = traj.get("frames", [])
        if not frames: continue
        t_arr = [fr.get("t", None) for fr in frames]
        x_arr = [fr.get("x", 0.0) for fr in frames]
        z_arr = [fr.get("z", 0.0) for fr in frames]
        g_arr = [fr.get("g", None) for fr in frames]
        drone_tracks[name] = {
            "t": np.array(t_arr, dtype=float) if (t_arr and t_arr[0] is not None) else None,
            "x": np.array(x_arr, dtype=float),
            "z": np.array(z_arr, dtype=float),
            "g": np.array(g_arr, dtype=float) if any(v is not None for v in g_arr) else None,
        }
elif "swarmState" in data:
    top_time = data.get("time", None)
    top_time = np.array(top_time, dtype=float) if isinstance(top_time, list) else None
    for entry in data["swarmState"]:
        name = str(entry.get("droneId", f"d{len(drone_tracks)}"))
        pos = entry.get("droneState", {}).get("position", [])
        if not pos: continue
        x_arr = [p.get("x", 0.0) for p in pos]
        z_arr = [p.get("z", 0.0) for p in pos]
        g_arr = [p.get("g", None) for p in pos]
        t_here = top_time if (top_time is not None and len(top_time) == len(x_arr)) else None
        drone_tracks[name] = {
            "t": t_here,
            "x": np.array(x_arr, dtype=float),
            "z": np.array(z_arr, dtype=float),
            "g": np.array(g_arr, dtype=float) if any(v is not None for v in g_arr) else None,
        }
else:
    raise ValueError("Unrecognized JSON layout (expected 'trajectories' or 'swarmState').")

if not drone_tracks:
    raise ValueError("No drone trajectories found.")

pts = [(0.0, 0.0)]
x, z = 0.0, 0.0
for dx, dz in REF_STEPS:
    x += dx; z += dz
    pts.append((x, z))
ref_poly = np.array(pts, dtype=float) * REF_SCALE

def closest_point_on_segment(p, a, b):
    ap = p - a
    ab = b - a
    ab2 = float(ab[0]*ab[0] + ab[1]*ab[1])
    if ab2 == 0.0:
        return a, float((p[0]-a[0])**2 + (p[1]-a[1])**2)
    t = (ap[0]*ab[0] + ap[1]*ab[1]) / ab2
    t = max(0.0, min(1.0, t))
    q = a + t*ab
    d2 = float((p[0]-q[0])**2 + (p[1]-q[1])**2)
    return q, d2

def dist_point_to_polyline(p, poly):
    best_d2 = float("inf")
    for i in range(len(poly)-1):
        _, d2 = closest_point_on_segment(p, poly[i], poly[i+1])
        if d2 < best_d2:
            best_d2 = d2
    return math.sqrt(best_d2)

use_time = any(drone_tracks[name]["t"] is not None for name in drone_tracks)

if use_time:
    bins = {}
    for name, d in drone_tracks.items():
        t = d["t"]; xarr = d["x"]; zarr = d["z"]; g = d.get("g", None)
        if t is None: continue
        for idx, (ti, xi, zi) in enumerate(zip(t, xarr, zarr)):
            if g is not None and not (g[idx] == 1):
                continue
            key = round(float(ti), 3)
            bins.setdefault(key, []).append((xi, zi))
    if not bins:
        for name, d in drone_tracks.items():
            t = d["t"]; xarr = d["x"]; zarr = d["z"]
            if t is None: continue
            for ti, xi, zi in zip(t, xarr, zarr):
                key = round(float(ti), 3)
                bins.setdefault(key, []).append((xi, zi))
    times = np.array(sorted(bins.keys()), dtype=float)
    centroid_x = np.array([np.mean([p[0] for p in bins[t]]) for t in times], dtype=float)
    centroid_z = np.array([np.mean([p[1] for p in bins[t]]) for t in times], dtype=float)
else:
    min_len = min(len(drone_tracks[name]["x"]) for name in drone_tracks)
    times = np.arange(min_len, dtype=float)
    xs, zs = [], []
    for f in range(min_len):
        pts = []
        for name, d in drone_tracks.items():
            xval = d["x"][f]; zval = d["z"][f]
            g = d.get("g", None)
            if g is not None and not (g[f] == 1):
                continue
            pts.append((xval, zval))
        if not pts:
            for name, d in drone_tracks.items():
                pts.append((d["x"][f], d["z"][f]))
        xs.append(np.mean([p[0] for p in pts]))
        zs.append(np.mean([p[1] for p in pts]))
    centroid_x = np.array(xs, dtype=float)
    centroid_z = np.array(zs, dtype=float)

centroid = np.column_stack([centroid_x, centroid_z])
centroid_err = np.array([dist_point_to_polyline(p, ref_poly) for p in centroid], dtype=float)

# ========= Average inter-agent distance (main group only if g provided) =========

def avg_pairwise_distance(points_xy):
    """Mean of all pairwise Euclidean distances among points (N,2). Returns np.nan if <2."""
    m = points_xy.shape[0]
    if m < 2:
        return np.nan
    # pairwise distances using broadcasting; take upper triangle
    diffs = points_xy[:, None, :] - points_xy[None, :, :]      # (m,m,2)
    dists = np.sqrt(np.sum(diffs * diffs, axis=-1))             # (m,m)
    iu = np.triu_indices(m, k=1)
    return float(dists[iu].mean())

if use_time:
    # Rebuild bins but keep all positions so we can compute pairwise distances
    bins_all = {}  # time -> list of (x,z) after g-filter
    for name, d in drone_tracks.items():
        t = d["t"]; xarr = d["x"]; zarr = d["z"]; g = d.get("g", None)
        if t is None:
            continue
        for idx, (ti, xi, zi) in enumerate(zip(t, xarr, zarr)):
            if g is not None and not (g[idx] == 1):
                continue
            key = round(float(ti), 3)
            bins_all.setdefault(key, []).append((xi, zi))

    # If g-filter removed everything at some times, fall back to include all drones at those times
    if not bins_all:
        for name, d in drone_tracks.items():
            t = d["t"]; xarr = d["x"]; zarr = d["z"]
            if t is None: continue
            for ti, xi, zi in zip(t, xarr, zarr):
                key = round(float(ti), 3)
                bins_all.setdefault(key, []).append((xi, zi))

    times_inter = np.array(sorted(bins_all.keys()), dtype=float)
    avg_interagent = []
    counts_used = []
    for key in times_inter:
        pts = np.array(bins_all[key], dtype=float)
        avg_interagent.append(avg_pairwise_distance(pts))
        counts_used.append(len(pts))
    avg_interagent = np.array(avg_interagent, dtype=float)

else:
    # Frame-index mode: compute per frame using available drones; apply g-filter if present
    min_len = min(len(drone_tracks[name]["x"]) for name in drone_tracks)
    times_inter = np.arange(min_len, dtype=float)
    avg_interagent = []
    counts_used = []
    for f in range(min_len):
        pts = []
        for name, d in drone_tracks.items():
            g = d.get("g", None)
            if g is not None and not (g[f] == 1):
                continue
            pts.append((d["x"][f], d["z"][f]))
        if len(pts) == 0:
            # fallback: include all drones at this frame
            pts = [(d["x"][f], d["z"][f]) for d in drone_tracks.values()]
        pts = np.array(pts, dtype=float)
        avg_interagent.append(avg_pairwise_distance(pts))
        counts_used.append(len(pts))
    avg_interagent = np.array(avg_interagent, dtype=float)

# Overall mean across timestamps (ignoring NaNs when only 0/1 drone in group)
overall_avg_interagent = float(np.nanmean(avg_interagent)) if avg_interagent.size else float("nan")
print(f"Average inter-agent distance over time: {overall_avg_interagent:.3f} m")

# ========= Average inter-agent distance for the WHOLE SWARM (ignore g) =========

def avg_pairwise_distance(points_xy):
    m = points_xy.shape[0]
    if m < 2:
        return np.nan
    diffs = points_xy[:, None, :] - points_xy[None, :, :]   # (m,m,2)
    dists = np.sqrt(np.sum(diffs * diffs, axis=-1))         # (m,m)
    iu = np.triu_indices(m, k=1)
    return float(dists[iu].mean())

if use_time:
    # Build bins without any g filtering
    bins_swarm = {}  # time -> list of (x,z) for all drones
    for name, d in drone_tracks.items():
        t = d["t"]; xarr = d["x"]; zarr = d["z"]
        if t is None: continue
        for ti, xi, zi in zip(t, xarr, zarr):
            key = round(float(ti), 3)
            bins_swarm.setdefault(key, []).append((xi, zi))

    times_swarm = np.array(sorted(bins_swarm.keys()), dtype=float)
    avg_interagent_swarm = []
    counts_swarm = []
    for key in times_swarm:
        pts = np.array(bins_swarm[key], dtype=float)
        avg_interagent_swarm.append(avg_pairwise_distance(pts))
        counts_swarm.append(len(pts))
    avg_interagent_swarm = np.array(avg_interagent_swarm, dtype=float)

else:
    # Frame-index mode: whole swarm each frame
    min_len = min(len(d["x"]) for d in drone_tracks.values())
    times_swarm = np.arange(min_len, dtype=float)
    avg_interagent_swarm = []
    counts_swarm = []
    for f in range(min_len):
        pts = np.array([(d["x"][f], d["z"][f]) for d in drone_tracks.values()], dtype=float)
        avg_interagent_swarm.append(avg_pairwise_distance(pts))
        counts_swarm.append(len(pts))
    avg_interagent_swarm = np.array(avg_interagent_swarm, dtype=float)

overall_avg_interagent_swarm = float(np.nanmean(avg_interagent_swarm)) if avg_interagent_swarm.size else float("nan")
print(f"Average inter-agent distance (WHOLE SWARM) over time: {overall_avg_interagent_swarm:.3f} m")

# --------- METRICS ---------
# total time
if len(centroid_err) > 0:
    if use_time:
        total_time_s = float(times[-1] - times[0])
    else:
        # try to infer from JSON if no per-frame times
        sample_hz = data.get("sampleHz", None)
        if isinstance(sample_hz, (int, float)) and sample_hz and sample_hz > 0:
            total_time_s = float((len(times) - 1) / sample_hz)
        else:
            total_time_s = float("nan")
else:
    total_time_s = 0.0

# average centroid-to-reference distance
avg_err_m = float(np.mean(centroid_err)) if len(centroid_err) else float("nan")

# survivors = drones with g==1 on their final recorded frame
survivors = 0
with_g = 0
for name, d in drone_tracks.items():
    g = d.get("g", None)
    if g is None or len(g) == 0:
        continue
    with_g += 1
    if int(g[-1]) == 1:
        survivors += 1

print("=== RUN METRICS ===")
print(f"Total time: {total_time_s:.2f} s")
print(f"Average centroid→reference distance: {avg_err_m:.3f} m")
print(f"Survived drones (final g==1): {survivors} / {with_g} (with g-labels), total drones: {len(drone_tracks)}")

# (optional) also write to a sidecar text file next to your PNGs
metrics_txt = Path(OUT_ERR_PNG).with_suffix(".metrics.txt")
metrics_txt.write_text(
    f"file: {INPUT_JSON}\n"
    f"scene: {scene}\n"
    f"total_time_s: {total_time_s:.3f}\n"
    f"avg_centroid_ref_dist_m: {avg_err_m:.6f}\n"
    f"survivors_final_g1: {survivors}\n"
    f"drones_with_g_label: {with_g}\n"
    f"total_drones: {len(drone_tracks)}\n"
)
print(f"(metrics saved to) {metrics_txt}")


plt.figure(figsize=(8, 8))
for name, d in drone_tracks.items():
    plt.plot(d["x"], d["z"], alpha=0.25)
plt.plot(centroid_x, centroid_z, linewidth=3, label="Swarm centroid (main group)")
plt.scatter([centroid_x[0]],[centroid_z[0]], s=50, marker="o")
plt.scatter([centroid_x[-1]],[centroid_z[-1]], s=50, marker="x")
plt.plot(ref_poly[:,0], ref_poly[:,1], linewidth=3, linestyle="--", label=f"Reference (×{REF_SCALE})")
plt.scatter(ref_poly[:,0], ref_poly[:,1], s=20)
plt.gca().set_aspect("equal", adjustable="box")
plt.xlabel("X (m)"); plt.ylabel("Z (m)")
plt.title(f"Trajectories & Centroid vs Reference — {scene}\nFile: {INPUT_JSON.name}")
plt.grid(True, alpha=0.3); plt.legend(loc="best")
plt.tight_layout(); plt.savefig(OUT_TRAJ_PNG, dpi=150); #plt.show()

plt.figure(figsize=(9, 5))
plt.plot(times, centroid_err)
plt.xlabel("Time (s)" if use_time else "Frame index")
plt.ylabel("Centroid cross-track error (m)")
plt.title("Centroid cross-track error vs time (main group only)")
plt.grid(True, alpha=0.3)
plt.tight_layout(); plt.savefig(OUT_ERR_PNG, dpi=150); #plt.show()

# Save + plot
# OUT_INTERDIST_PNG = "outputs/average_interagent_distance.png"
# plt.figure(figsize=(9, 5))
# plt.plot(times_inter, avg_interagent)
# plt.xlabel("Time (s)" if use_time else "Frame index")
# plt.ylabel("Average inter-agent distance (m)")
# plt.title("Average inter-agent distance vs time (main group when available)")
# plt.grid(True, alpha=0.3)
# plt.tight_layout()
# plt.savefig(OUT_INTERDIST_PNG, dpi=150)
# plt.show()

# --------- PLOT: overlay main-group vs whole swarm ---------
OUT_INTERDIST_BOTH_PNG = "outputs/average_interagent_distance_both.png"
plt.figure(figsize=(9, 5))
# main group you already computed: times_inter, avg_interagent
plt.plot(times_inter, avg_interagent, label="Main group", linewidth=2)
plt.plot(times_swarm, avg_interagent_swarm, label="Whole swarm", linestyle="--")
plt.xlabel("Time (s)" if use_time else "Frame index")
plt.ylabel("Average inter-agent distance (m)")
plt.title("Average inter-agent distance vs time")
plt.grid(True, alpha=0.3)
plt.legend(loc="best")
plt.tight_layout()
plt.savefig(OUT_INTERDIST_BOTH_PNG, dpi=150)
plt.show()

print("Saved:", OUT_INTERDIST_PNG)
OUT_TRAJ_PNG, OUT_ERR_PNG, str(INPUT_JSON)


