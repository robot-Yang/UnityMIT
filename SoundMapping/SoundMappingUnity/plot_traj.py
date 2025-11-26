#!/usr/bin/env python3
"""
Process swarm trajectory JSON and plot:
  1) X–Z trajectories with a 0.3× reference path
  2) Average cross-track error vs time

Also saves per-drone cross-track errors to CSV.

Works with either:
  A) {"trajectories":[{"name"/"id", "frames":[{"t","x","y","z"}, ...]}, ...]}
  B) {"swarmState":[{"droneId", "droneState":{"position":[{"x","y","z"}, ...]}}, ...], "time":[...]}
"""

import json, math, os
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# ------------------ CONFIG ------------------
input_path = "/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/FPVObs_2_H_NO_2025-10-24-16-26-44.json"     # <— set this (absolute or relative path)
out_dir    = "outputs"            # folder to write PNGs/CSV
ref_scale  = 0.3                  # 0.3× scaling requested
legend_cap = 12                   # max drones to label in legend
# --------------------------------------------

Path(out_dir).mkdir(parents=True, exist_ok=True)

# ----- Load JSON -----
with open(input_path, "r") as f:
    data = json.load(f)

scene      = data.get("scene", "Unknown Scene")
sample_hz  = data.get("sampleHz", None)  # if present (Format A)

# ----- Parse drone trajectories (robust to two common formats) -----
drones = []  # each: { 'id': <name/id>, 't': np.array | None, 'x': np.array, 'z': np.array }

if "trajectories" in data:  # Format A
    for i, traj in enumerate(data["trajectories"]):
        name = traj.get("name", f"id:{traj.get('id', i)}")
        frames = traj.get("frames", [])
        if not frames:
            continue
        t = np.array([fr["t"] for fr in frames], dtype=float) if "t" in frames[0] else None
        x = np.array([fr["x"] for fr in frames], dtype=float)
        z = np.array([fr["z"] for fr in frames], dtype=float)
        drones.append({"id": name, "t": t, "x": x, "z": z})

elif "swarmState" in data:  # Format B (like saveInfoToJSON)
    # Optional time array at top-level
    global_time = np.array(data.get("time", []), dtype=float) if "time" in data else None
    for entry in data["swarmState"]:
        did = entry.get("droneId", "unknown")
        pos = entry.get("droneState", {}).get("position", [])
        if not pos:
            continue
        x = np.array([p.get("x", 0.0) for p in pos], dtype=float)
        z = np.array([p.get("z", 0.0) for p in pos], dtype=float)
        t = global_time if (global_time is not None and len(global_time) == len(x)) else None
        drones.append({"id": f"{did}", "t": t, "x": x, "z": z})
else:
    raise ValueError("Unrecognized JSON layout: expected 'trajectories' or 'swarmState'.")

if not drones:
    raise ValueError("No drone trajectories found in the file.")

# ----- Reference polyline (X–Z), scaled by 0.3 -----
# Start at (0,0), then +200m X, +340m Z, -210m X, +70m Z
ref_points = np.array([
    [0.0,   0.0],
    [200.0, 0.0],
    [200.0, 340.0],
    [-10.0, 340.0],  # 200 - 210 = -10
    [-10.0, 410.0],
], dtype=float) * ref_scale   # (N,2), columns: X,Z

# Precompute segments for distance calculations
seg_starts = ref_points[:-1]                     # (M,2)
seg_ends   = ref_points[1:]
seg_vecs   = seg_ends - seg_starts               # (M,2)
seg_lens2  = np.sum(seg_vecs**2, axis=1)         # (M,)

def point_to_polyline_distance(px: float, pz: float) -> float:
    """Shortest distance from point (px,pz) to piecewise-linear ref path."""
    p = np.array([px, pz], dtype=float)
    best = float("inf")
    for s, v, L2 in zip(seg_starts, seg_vecs, seg_lens2):
        if L2 == 0:
            d = np.linalg.norm(p - s)
        else:
            t = np.dot(p - s, v) / L2
            t = max(0.0, min(1.0, t))  # clamp to segment
            proj = s + t * v
            d = np.linalg.norm(p - proj)
        if d < best:
            best = d
    return float(best)

# ----- Build a common time grid (if time is available) -----
# If some drones have per-frame timestamps, align by time; otherwise align by index.
have_any_time = any(d["t"] is not None for d in drones)

if have_any_time:
    # Determine dt: prefer sample_hz; else infer from medians
    if sample_hz and sample_hz > 0:
        dt = 1.0 / float(sample_hz)
    else:
        dts = []
        for d in drones:
            t = d["t"]
            if t is not None and len(t) >= 2:
                dts.append(np.median(np.diff(t)))
        dt = float(np.median(dts)) if dts else 1.0

    # Overlap window across drones
    t_min = max(d["t"][0] for d in drones if d["t"] is not None)
    t_max = min(d["t"][-1] for d in drones if d["t"] is not None)
    if t_max <= t_min:
        # fallback: overall min/max
        t_min = min(d["t"][0] for d in drones if d["t"] is not None)
        t_max = max(d["t"][-1] for d in drones if d["t"] is not None)

    time_grid = np.arange(t_min, t_max + 1e-9, dt)

    # Interpolate each drone onto grid and compute per-drone error time series
    per_drone_err_series = []
    for d in drones:
        t = d["t"]
        x = d["x"]
        z = d["z"]
        if t is None or len(t) < 2:
            # no time info: approximate with constant over grid
            xi = np.full_like(time_grid, x[0] if len(x) else np.nan, dtype=float)
            zi = np.full_like(time_grid, z[0] if len(z) else np.nan, dtype=float)
        else:
            # only within coverage
            valid = (time_grid >= t[0]) & (time_grid <= t[-1])
            xi = np.full_like(time_grid, np.nan, dtype=float)
            zi = np.full_like(time_grid, np.nan, dtype=float)
            xi[valid] = np.interp(time_grid[valid], t, x)
            zi[valid] = np.interp(time_grid[valid], t, z)

        errs = np.array([point_to_polyline_distance(a, b) if (not math.isnan(a) and not math.isnan(b)) else np.nan
                         for a, b in zip(xi, zi)], dtype=float)
        per_drone_err_series.append(errs)

    err_stack = np.vstack(per_drone_err_series)           # (N_drones, N_times)
    avg_err   = np.nanmean(err_stack, axis=0)             # (N_times,)
    n_used    = np.sum(~np.isnan(err_stack), axis=0)

else:
    # Align by index (shortest length)
    min_len = min(len(d["x"]) for d in drones)
    time_grid = np.arange(min_len)  # index as "time"
    per_drone_err_series = []
    for d in drones:
        x = d["x"][:min_len]
        z = d["z"][:min_len]
        errs = np.array([point_to_polyline_distance(xi, zi) for xi, zi in zip(x, z)], dtype=float)
        per_drone_err_series.append(errs)
    err_stack = np.vstack(per_drone_err_series)
    avg_err   = np.mean(err_stack, axis=0)
    n_used    = np.full(min_len, len(drones), dtype=int)

# ----- Plot 1: X–Z trajectories + reference -----
plt.figure(figsize=(8, 8))
for i, d in enumerate(drones):
    plt.plot(d["x"], d["z"], label=d["id"] if i < legend_cap else None)
plt.plot(ref_points[:,0], ref_points[:,1], linestyle="--", linewidth=3, label=f"Reference (x{ref_scale})")
plt.scatter(ref_points[:,0], ref_points[:,1], s=25)
plt.xlabel("X (m)")
plt.ylabel("Z (m)")
plt.title(f"Trajectories (X–Z) — {scene}")
plt.axis("equal")
plt.grid(True, alpha=0.3)
plt.legend(loc="best", fontsize=8, ncol=2)
traj_png = os.path.join(out_dir, "traj_xz_with_reference.png")
plt.tight_layout()
plt.savefig(traj_png, dpi=150)
plt.close()

# ----- Plot 2: Average cross-track error vs time -----
plt.figure(figsize=(9, 5))
plt.plot(time_grid, avg_err, marker="o", linewidth=1.5)
plt.xlabel("Time (s)" if have_any_time else "Frame index")
plt.ylabel("Average cross-track error (m)")
plt.title("Average cross-track error vs time")
plt.grid(True, alpha=0.3)
err_png = os.path.join(out_dir, "avg_crosstrack_error_vs_time.png")
plt.tight_layout()
plt.savefig(err_png, dpi=150)
plt.close()

# ----- Save per-drone errors to CSV (aligned to time_grid) -----
rows = []
for d_idx, d in enumerate(drones):
    errs = err_stack[d_idx]
    for ti, e in zip(time_grid, errs):
        rows.append({
            ("time_s" if have_any_time else "frame"): float(ti),
            "drone_id": d["id"],
            "cross_track_error_m": (None if math.isnan(e) else float(e))
        })
df = pd.DataFrame(rows)
csv_path = os.path.join(out_dir, "per_drone_crosstrack_error.csv")
df.to_csv(csv_path, index=False)

print("Wrote:")
print(" -", traj_png)
print(" -", err_png)
print(" -", csv_path)
