# === Averages limited to the Run window (start..stop) + robust end-state classification ===
import json, math, glob, os
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# ---------------- Config ----------------
REF_SCALE = 0.3
REF_STEPS = [
    (0, 140), (-140, 0), (0, 100), (100, 0),
    (0, 160), (-100, 0), (0, 100), (-140, 0),
    (0, -160), (-200, 0), (0, 100), (100, 0),
]

# How forgiving to be when deciding if a drone was "present at stop"
# (covers sampling quantization and scene-switch save slop).
SCENE_SWITCH_GRACE_S = 1.0

OUT_DIR = Path("outputs"); OUT_DIR.mkdir(parents=True, exist_ok=True)
OUT_TRAJ_PNG = str(OUT_DIR / "one_script_trajectories.png")
OUT_ERR_PNG  = str(OUT_DIR / "one_script_centroid_error.png")
OUT_INTERDIST_BOTH_PNG = str(OUT_DIR / "average_interagent_distance_both.png")

# -------- Select input JSON (edit as needed) --------
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251031_232835_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251101_133056_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251101_205912_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251101_223938_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251101_225258_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251102_002820_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251102_004342_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251102_005706_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251102_010220_traj.json"), key=os.path.getmtime, reverse=True)

# shuhang without haptic
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang/Setup_H_NO_20251102_134927_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang/Setup_H_NO_20251102_135136_traj.json"), key=os.path.getmtime, reverse=True)
#### candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang/Setup_H_NO_20251102_140203_traj.json"), key=os.path.getmtime, reverse=True)

# shuhang with haptic
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang/Setup_H_NO_20251102_140450_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang/Setup_H_NO_20251102_140704_traj.json"), key=os.path.getmtime, reverse=True)

# fuda with haptic
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Fuda/Setup_H_NO_20251102_150636_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Fuda/Setup_H_NO_20251102_150935_traj.json"), key=os.path.getmtime, reverse=True) 

# fuda without haptic
### candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Fuda/Setup_H_NO_20251102_151250_traj.json"), key=os.path.getmtime, reverse=True) 
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Fuda/Setup_H_NO_20251102_151520_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Fuda/Setup_H_NO_20251102_151718_traj.json"), key=os.path.getmtime, reverse=True)

# test
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/1103/Setup_H_NO_20251103_145134_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/1103/Setup_H_NO_20251103_145314_traj.json"), key=os.path.getmtime, reverse=True)


if not candidates:
    raise FileNotFoundError("No JSON files found. Update the candidates glob.")
INPUT_JSON = Path(candidates[0])

with INPUT_JSON.open("r") as f:
    data = json.load(f)

scene = data.get("scene", data.get("level", "Unknown Scene"))
sample_hz = data.get("sampleHz", None)
# If your file didn’t record sampleHz, set a fallback here:
if not isinstance(sample_hz, (int, float)) or sample_hz <= 0:
    sample_hz = 5.0

# -------- Parse drones --------
drone_tracks = {}
if "trajectories" in data:
    for i, traj in enumerate(data["trajectories"]):
        name   = traj.get("name", f"id:{traj.get('id', i)}")
        frames = traj.get("frames", [])
        if not frames: 
            continue
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
        pos  = entry.get("droneState", {}).get("position", [])
        if not pos: 
            continue
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

# -------- Reference path --------
pts = [(0.0, 0.0)]
x, z = 0.0, 0.0
for dx, dz in REF_STEPS:
    x += dx; z += dz
    pts.append((x, z))
ref_poly = np.array(pts, dtype=float) * REF_SCALE

def closest_point_on_segment(p, a, b):
    ap = p - a; ab = b - a
    ab2 = float(ab[0]*ab[0] + ab[1]*ab[1])
    if ab2 == 0.0:
        return a, float((p[0]-a[0])**2 + (p[1]-a[1])**2)
    t = (ap[0]*ab[0] + ap[1]*ab[1]) / ab2; t = max(0.0, min(1.0, t))
    q = a + t*ab; d2 = float((p[0]-q[0])**2 + (p[1]-q[1])**2)
    return q, d2

def dist_point_to_polyline(p, poly):
    best_d2 = float("inf")
    for i in range(len(poly)-1):
        _, d2 = closest_point_on_segment(p, poly[i], poly[i+1])
        if d2 < best_d2: best_d2 = d2
    return math.sqrt(best_d2)

# -------- Centroid (main group if g present) --------
use_time = any(drone_tracks[name]["t"] is not None for name in drone_tracks)

if use_time:
    bins = {}
    for name, d in drone_tracks.items():
        t = d["t"]; xarr = d["x"]; zarr = d["z"]; g = d.get("g", None)
        if t is None: 
            continue
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
    times = np.arange(min_len, dtype=float)  # frame index
    xs, zs = [], []
    for f in range(min_len):
        pts_f = []
        for name, d in drone_tracks.items():
            g = d.get("g", None)
            if g is not None and not (g[f] == 1): continue
            pts_f.append((d["x"][f], d["z"][f]))
        if not pts_f:
            for name, d in drone_tracks.items():
                pts_f.append((d["x"][f], d["z"][f]))
        xs.append(np.mean([p[0] for p in pts_f]))
        zs.append(np.mean([p[1] for p in pts_f]))
    centroid_x = np.array(xs, dtype=float)
    centroid_z = np.array(zs, dtype=float)

centroid = np.column_stack([centroid_x, centroid_z])
centroid_err = np.array([dist_point_to_polyline(p, ref_poly) for p in centroid], dtype=float)

# -------- Average inter-agent distance (main + whole) --------
def avg_pairwise_distance(points_xy):
    m = points_xy.shape[0]
    if m < 2: return np.nan
    diffs = points_xy[:, None, :] - points_xy[None, :, :]
    dists = np.sqrt(np.sum(diffs * diffs, axis=-1))
    iu = np.triu_indices(m, k=1)
    return float(dists[iu].mean())

# main group
if use_time:
    bins_all = {}
    for name, d in drone_tracks.items():
        t = d["t"]; xarr = d["x"]; zarr = d["z"]; g = d.get("g", None)
        if t is None: continue
        for idx, (ti, xi, zi) in enumerate(zip(t, xarr, zarr)):
            if g is not None and not (g[idx] == 1): continue
            key = round(float(ti), 3)
            bins_all.setdefault(key, []).append((xi, zi))
    if not bins_all:
        for name, d in drone_tracks.items():
            t = d["t"]; xarr = d["x"]; zarr = d["z"]
            if t is None: continue
            for ti, xi, zi in zip(t, xarr, zarr):
                key = round(float(ti), 3)
                bins_all.setdefault(key, []).append((xi, zi))
    times_inter = np.array(sorted(bins_all.keys()), dtype=float)
    avg_interagent = np.array([avg_pairwise_distance(np.array(bins_all[k], dtype=float)) for k in times_inter], dtype=float)
else:
    min_len = min(len(drone_tracks[name]["x"]) for name in drone_tracks)
    times_inter = np.arange(min_len, dtype=float)
    avg_interagent = []
    for f in range(min_len):
        pts = []
        for name, d in drone_tracks.items():
            g = d.get("g", None)
            if g is not None and not (g[f] == 1): continue
            pts.append((d["x"][f], d["z"][f]))
        if len(pts) == 0:
            pts = [(d["x"][f], d["z"][f]) for d in drone_tracks.values()]
        avg_interagent.append(avg_pairwise_distance(np.array(pts, dtype=float)))
    avg_interagent = np.array(avg_interagent, dtype=float)

# whole swarm
if use_time:
    bins_swarm = {}
    for name, d in drone_tracks.items():
        t = d["t"]; xarr = d["x"]; zarr = d["z"]
        if t is None: continue
        for ti, xi, zi in zip(t, xarr, zarr):
            key = round(float(ti), 3)
            bins_swarm.setdefault(key, []).append((xi, zi))
    times_swarm = np.array(sorted(bins_swarm.keys()), dtype=float)
    avg_interagent_swarm = np.array([avg_pairwise_distance(np.array(bins_swarm[k], dtype=float)) for k in times_swarm], dtype=float)
else:
    min_len = min(len(d["x"]) for d in drone_tracks.values())
    times_swarm = np.arange(min_len, dtype=float)
    avg_interagent_swarm = []
    for f in range(min_len):
        pts = np.array([(d["x"][f], d["z"][f]) for d in drone_tracks.values()], dtype=float)
        avg_interagent_swarm.append(avg_pairwise_distance(pts))
    avg_interagent_swarm = np.array(avg_interagent_swarm, dtype=float)

# -------- Read Run start/stop and build masks --------
trial = None
trials = data.get("trials", [])
if isinstance(trials, list) and trials:
    runs = [t for t in trials if t.get("label")=="Run" and t.get("endGameTime",0)>t.get("startGameTime",0)]
    cand = runs if runs else [t for t in trials if t.get("endGameTime",0)>t.get("startGameTime",0)]
    if cand:
        trial = max(cand, key=lambda t: t["endGameTime"] - t["startGameTime"])
t0 = float(trial["startGameTime"]) if trial else None
t1 = float(trial["endGameTime"])   if trial else None

def game_time_to_axis_x(t_game, use_time, sample_hz):
    if t_game is None: return None
    if use_time: return t_game
    if sample_hz: return t_game * sample_hz
    return None

x0_cte = game_time_to_axis_x(t0, use_time, sample_hz)
x1_cte = game_time_to_axis_x(t1, use_time, sample_hz)

# Masks for run window (inclusive) — fall back to "all True" if no trial
if t0 is not None and t1 is not None:
    mask_cte = (times >= x0_cte) & (times <= x1_cte)
else:
    mask_cte = np.ones_like(times, dtype=bool)

if t0 is not None and t1 is not None:
    x0_inter = game_time_to_axis_x(t0, use_time, sample_hz)
    x1_inter = game_time_to_axis_x(t1, use_time, sample_hz)
    mask_inter_main  = (times_inter  >= x0_inter) & (times_inter  <= x1_inter)
    mask_inter_swarm = (times_swarm >= x0_inter) & (times_swarm <= x1_inter)
else:
    mask_inter_main  = np.ones_like(times_inter, dtype=bool)
    mask_inter_swarm = np.ones_like(times_swarm, dtype=bool)

# -------- End-state classification --------
def last_observed_game_time(d, sample_hz):
    """Return the last game-time we can infer for this drone."""
    t = d.get("t")
    if t is not None and len(t) > 0:
        return float(t[-1])
    # derive from length if we have no per-sample times
    if sample_hz and sample_hz > 0:
        return (len(d.get("x", [])) - 1) / float(sample_hz) if len(d.get("x", [])) > 0 else float("nan")
    return float("nan")

grace_s = max(1.0/float(sample_hz), SCENE_SWITCH_GRACE_S) if sample_hz else SCENE_SWITCH_GRACE_S

per_drone_status = {}  # name -> {'status': str, 't_last': float, 'g_last': int}
# statuses: 'survivor', 'disconnected_at_end', 'crashed_or_vanished_early', 'vanished_while_disconnected'

for name, d in drone_tracks.items():
    g = d.get("g", None)
    t_last = last_observed_game_time(d, sample_hz)
    g_last = int(g[-1]) if (g is not None and len(g) > 0) else -1

    if t0 is None or t1 is None or np.isnan(t_last):
        # no run window -> simple heuristic
        status = 'survivor' if g_last == 1 else 'disconnected_at_end'
    else:
        present_at_stop = (t_last >= (t1 - grace_s))
        if present_at_stop:
            status = 'survivor' if g_last == 1 else 'disconnected_at_end'
        else:
            # left early:
            if g_last == 1:
                # in main group at last sighting -> crashed/vanished early
                status = 'crashed_or_vanished_early'
            else:
                status = 'vanished_while_disconnected'

    per_drone_status[name] = dict(status=status, t_last=t_last, g_last=g_last)

# Survivors = present at stop & g_last==1
survivors = sum(1 for s in per_drone_status.values() if s['status']=='survivor')
with_g    = sum(1 for d in drone_tracks.values() if d.get("g", None) is not None and len(d["g"])>0)

# -------- METRICS (averages limited to Run window) --------
# total time (as before)
if len(centroid_err) > 0:
    if use_time:
        total_time_s = float(times[-1] - times[0])
    else:
        total_time_s = float((len(times) - 1) / sample_hz) if len(times)>1 else 0.0
else:
    total_time_s = 0.0

# Average centroid→reference distance (run mask)
avg_err_m = float(np.mean(centroid_err[mask_cte])) if np.any(mask_cte) else float("nan")

# Run total spent time
run_total_spent_time_s = float(t1 - t0) if (t0 is not None and t1 is not None) else float(total_time_s)

# Average inter-agent distances (run masks)
avg_interagent_main_overall  = float(np.nanmean(avg_interagent[mask_inter_main]))   if avg_interagent.size else float("nan")
avg_interagent_swarm_overall = float(np.nanmean(avg_interagent_swarm[mask_inter_swarm])) if avg_interagent_swarm.size else float("nan")

print("=== RUN METRICS (averages restricted to Run window) ===")
if t0 is not None and t1 is not None:
    print(f"Run window (game time): start={t0:.3f}s, stop={t1:.3f}s, total={run_total_spent_time_s:.3f}s")
else:
    print("No Run window found; using full extent for masks.")
print(f"Average centroid→reference distance (Run): {avg_err_m:.3f} m")
print(f"Average inter-agent distance (main, Run): {avg_interagent_main_overall:.3f} m")
print(f"Average inter-agent distance (swarm, Run): {avg_interagent_swarm_overall:.3f} m")
print(f"Survived drones (present at stop & final g==1): {survivors} / {with_g} (with g-labels), total drones: {len(drone_tracks)}")

# Debug: print each drone’s last time and status (helps verify cases like Drone4)
print("\n--- Drone end-state summary ---")
for name in sorted(drone_tracks.keys()):
    s = per_drone_status[name]
    print(f"{name:>12s}  t_last={s['t_last']:.6f}s   g_last={s['g_last']}   status={s['status']}")

# Sidecar metrics
metrics_txt = Path(OUT_ERR_PNG).with_suffix(".metrics.txt")
metrics_txt.write_text(
    f"file: {INPUT_JSON}\n"
    f"scene: {scene}\n"
    f"total_time_s: {total_time_s:.3f}\n"
    f"avg_centroid_ref_dist_m: {avg_err_m:.6f}\n"
    f"survivors_final_g1_present_at_stop: {survivors}\n"
    f"drones_with_g_label: {with_g}\n"
    f"total_drones: {len(drone_tracks)}\n"
    f"run_total_spent_time_s: {run_total_spent_time_s:.6f}\n"
    f"avg_interagent_main_overall_m: {avg_interagent_main_overall:.6f}\n"
    f"avg_interagent_swarm_overall_m: {avg_interagent_swarm_overall:.6f}\n"
)

# -------- Plot helpers --------
def game_time_to_axis_x_val(t_game):
    return game_time_to_axis_x(t_game, use_time, sample_hz)

x0_line = game_time_to_axis_x_val(t0)
x1_line = game_time_to_axis_x_val(t1)

# 1) Trajectories (X-Z)
plt.figure(figsize=(8, 8))
for name, d in drone_tracks.items():
    # Path
    plt.plot(d["x"], d["z"], alpha=0.25)
    # Endpoint marker by status
    if len(d["x"]) > 0:
        end_x, end_z = d["x"][-1], d["z"][-1]
        st = per_drone_status[name]['status']
        if st == 'survivor':
            plt.scatter(end_x, end_z, s=36, marker="o")
        elif st == 'disconnected_at_end':
            plt.scatter(end_x, end_z, s=50, marker="^")
        elif st == 'crashed_or_vanished_early':
            plt.scatter(end_x, end_z, s=60, marker="X")
        else:  # vanished_while_disconnected
            plt.scatter(end_x, end_z, s=36, marker="s")

# centroid path + start/end
plt.plot(centroid_x, centroid_z, linewidth=3, label="Swarm centroid (main group)")
plt.scatter([centroid_x[0]],[centroid_z[0]], s=50, marker="o", label="Centroid start")
plt.scatter([centroid_x[-1]],[centroid_z[-1]], s=50, marker="x", label="Centroid end")

# reference polyline
plt.plot(ref_poly[:,0], ref_poly[:,1], linewidth=3, linestyle="--", label=f"Reference (×{REF_SCALE})")
plt.scatter(ref_poly[:,0], ref_poly[:,1], s=20)

plt.gca().set_aspect("equal", adjustable="box")
plt.xlabel("X (m)"); plt.ylabel("Z (m)")
plt.title(f"Trajectories & Centroid vs Reference — {scene}\nFile: {INPUT_JSON.name}")
plt.grid(True, alpha=0.3)

# Legend entries for endpoint semantics
legend_elems = [
    Line2D([0],[0], marker='o', linestyle='None', label='Endpoint: survivor'),
    Line2D([0],[0], marker='^', linestyle='None', label='Endpoint: disconnected@stop'),
    Line2D([0],[0], marker='X', linestyle='None', label='Endpoint: crashed/vanished early'),
    Line2D([0],[0], marker='s', linestyle='None', label='Endpoint: vanished while disconnected'),
]
h0, l0 = plt.gca().get_legend_handles_labels()
plt.legend(handles=legend_elems + h0, loc="best")

plt.tight_layout(); plt.savefig(OUT_TRAJ_PNG, dpi=150)

# 2) Centroid cross-track error vs time (with Run shading)
plt.figure(figsize=(9, 5))
plt.plot(times, centroid_err, label="Centroid CTE")
if x0_line is not None and x1_line is not None and x1_line >= x0_line:
    plt.axvspan(x0_line, x1_line, alpha=0.15, label=f"Run window ({run_total_spent_time_s:.2f}s)")
    plt.axvline(x0_line, linestyle="--"); plt.axvline(x1_line, linestyle="--")
plt.xlabel("Time (s)" if use_time else f"Frame index (~{sample_hz:.1f} Hz)")
plt.ylabel("Centroid cross-track error (m)")
plt.title("Centroid cross-track error vs time (main group only)")
plt.grid(True, alpha=0.3); plt.legend(loc="best")
plt.tight_layout(); plt.savefig(OUT_ERR_PNG, dpi=150)

# 3) Avg inter-agent distance (main vs swarm) with Run shading
plt.figure(figsize=(9, 5))
plt.plot(times_inter, avg_interagent, label="Main group", linewidth=2)
plt.plot(times_swarm, avg_interagent_swarm, label="Whole swarm", linestyle="--")
if x0_line is not None and x1_line is not None and x1_line >= x0_line:
    plt.axvspan(x0_line, x1_line, alpha=0.15, label=f"Run window ({run_total_spent_time_s:.2f}s)")
    plt.axvline(x0_line, linestyle="--"); plt.axvline(x1_line, linestyle="--")
plt.xlabel("Time (s)" if use_time else f"Frame index (~{sample_hz:.1f} Hz)")
plt.ylabel("Average inter-agent distance (m)")
plt.title("Average inter-agent distance vs time")
plt.grid(True, alpha=0.3); plt.legend(loc="best")
plt.tight_layout(); plt.savefig(OUT_INTERDIST_BOTH_PNG, dpi=150)
plt.show()

print("Saved:", OUT_TRAJ_PNG, OUT_ERR_PNG, OUT_INTERDIST_BOTH_PNG)
