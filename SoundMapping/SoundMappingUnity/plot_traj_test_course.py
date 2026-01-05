"""
Compare audio vs no-audio runs across subjects.

Assumes trajectory folders inside Assets/Data/default/Trajectories/ are named
like `<subject>_sound` and `<subject>_no_sound`, each containing:
  - stars.json (star pickups)
  - a swarm trajectory JSON (any other *.json file in the folder)

For each run we collect:
  - total/run time (Run window if available, else inferred duration)
  - number of stars (len(records))
  - number of survivor drones (g==1 at end and present at stop)

Plots three scatter charts with x-axis categories ["No sound", "Sound"] and
one point per subject per condition.
"""
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import transforms
import numpy as np

BASE_DIR = Path("Assets/Data/default/Trajectories")
OUT_DIR = Path("../../../../Results/plots")
OUT_DIR.mkdir(parents=True, exist_ok=True)
JSON_OUTPUT_PATH = Path("../../../../Results/run_metrics.json")

# Batch filtering: set INCLUDE_ALL_BATCH = False and BATCH_ID = "B3" (example)
# to only include folders starting with that batch prefix (before the first underscore).
INCLUDE_ALL_BATCH = True
BATCH_ID = "0"

# Interactive trajectory preview (set batch/uid/condition to pick a run)
TRAJ_BATCH = "B1"
TRAJ_UID = "0"
TRAJ_CONDITION = "no_sound"  # "sound" or "no_sound"
ENABLE_TRAJECTORY_SLIDER = True
# View window for the interactive slider (meters)
CORRIDOR_WIDTH = 30.0  # initial width; height is 1.5× this value in the viewer
OBSTACLE_JSON_DEFAULT = Path("Assets/Data/default/ObstacleCourse/TestCourse.json")
OBSTACLE_JSON_BATCH = {
    "B3": Path("Assets/Data/default/ObstacleCourse/SimpleCourse.json"),
}
# Keep references to interactive figures/widgets alive
_SLIDER_FIGS = []
DELTA_BIN_INIT = 10
DELTA_BIN_MIN = 5
DELTA_BIN_MAX = 50
DELTA_BIN_LIMITS = {
    "run_time_s": (20, 1, 50),  # (init, min, max)
    "stars": (10, 1, 30),
    "survivors": (1, 1, 12),
}

# Composite score weights (tweak as needed)
SCORE_TIME_W = -0.1       # negative: shorter time -> higher score
SCORE_STARS_W = 0.5       # reward collected stars
SCORE_LOST_W = -2      # penalty per lost drone

SCENE_SWITCH_GRACE_S = 1.0
DEFAULT_SAMPLE_HZ = 5.0


def _target_batch_prefix():
    if INCLUDE_ALL_BATCH:
        return None
    batch = BATCH_ID.strip()
    if not batch:
        return None
    if batch and not batch.startswith("B"):
        batch = f"B{batch}"
    return batch


def _output_png_path():
    batch = _target_batch_prefix()
    if batch is None:
        name = "metrics_comparison"
    else:
        name = f"metrics_comparison_{batch}"
    return OUT_DIR / f"{name}.png"


OUT_PNG = _output_png_path()


def _obstacle_json_path_for_batch(batch: str | None):
    if batch:
        batch = batch.strip()
        if batch in OBSTACLE_JSON_BATCH:
            return OBSTACLE_JSON_BATCH[batch]
    return OBSTACLE_JSON_DEFAULT


def _load_json(path: Path):
    with path.open("r") as f:
        return json.load(f)


def _parse_drone_tracks(data):
    tracks = []
    if "trajectories" in data:
        print("Trajectory detected")
        for i, traj in enumerate(data["trajectories"]):
            frames = traj.get("frames", [])
            if not frames:
                continue
            name = traj.get("name", f"id:{traj.get('id', i)}")
            t_arr = [fr.get("t", None) for fr in frames]
            x_arr = [fr.get("x", 0.0) for fr in frames]
            z_arr = [fr.get("z", 0.0) for fr in frames]
            g_arr = [fr.get("g", None) for fr in frames]
            e_arr = [fr.get("e", None) for fr in frames]
            tracks.append(
                dict(
                    name=name,
                    id=traj.get("id", None),
                    t=np.array(t_arr, dtype=float) if (t_arr and t_arr[0] is not None) else None,
                    x=np.array(x_arr, dtype=float),
                    z=np.array(z_arr, dtype=float),
                    g=np.array(g_arr, dtype=float) if any(v is not None for v in g_arr) else None,
                    e=np.array(e_arr, dtype=float) if any(v is not None for v in e_arr) else None,
                    embodied=bool(traj.get("embodied", False) or traj.get("isEmbodied", False)),
                )
            )
    elif "swarmState" in data:
        top_time = data.get("time", None)
        top_time = np.array(top_time, dtype=float) if isinstance(top_time, list) else None
        for entry in data["swarmState"]:
            pos = entry.get("droneState", {}).get("position", [])
            if not pos:
                continue
            name = str(entry.get("droneId", f"d{len(tracks)}"))
            x_arr = [p.get("x", 0.0) for p in pos]
            z_arr = [p.get("z", 0.0) for p in pos]
            g_arr = [p.get("g", None) for p in pos]
            e_arr = [p.get("e", None) for p in pos]
            t_here = top_time if (top_time is not None and len(top_time) == len(x_arr)) else None
            tracks.append(
                dict(
                    name=name,
                    id=entry.get("droneId", None),
                    t=t_here,
                    x=np.array(x_arr, dtype=float),
                    z=np.array(z_arr, dtype=float),
                    g=np.array(g_arr, dtype=float)
                    if any(v is not None for v in g_arr)
                    else None,
                    e=np.array(e_arr, dtype=float) if any(v is not None for v in e_arr) else None,
                    embodied=bool(entry.get("isEmbodied", False)),
                )
            )
    else:
        raise ValueError("Unrecognized JSON layout (expected 'trajectories' or 'swarmState').")
    return tracks


def _infer_total_time(tracks, sample_hz):
    """Rough duration based on available times or frame counts."""
    if not tracks:
        return 0.0
    min_t = None
    max_t = 0.0
    for tr in tracks:
        t_arr = tr.get("t")
        if t_arr is not None and len(t_arr) > 0:
            start = float(t_arr[0])
            end = float(t_arr[-1])
        else:
            n = len(tr.get("x", []))
            if n == 0:
                continue
            start = 0.0
            end = (n - 1) / float(sample_hz) if sample_hz else float(n - 1)
        min_t = end if min_t is None else min(min_t, start)
        max_t = max(max_t, end)
    return max(0.0, max_t - (min_t if min_t is not None else 0.0))


def _last_observed_game_time(tr, sample_hz):
    t = tr.get("t")
    if t is not None and len(t) > 0:
        return float(t[-1])
    n = len(tr.get("x", []))
    if n == 0:
        return float("nan")
    return (n - 1) / float(sample_hz) if sample_hz else float("nan")


def _classify_survivors(tracks, t0, t1, sample_hz):
    grace_s = max(1.0 / float(sample_hz), SCENE_SWITCH_GRACE_S) if sample_hz else SCENE_SWITCH_GRACE_S
    survivors = 0
    with_g = 0
    for tr in tracks:
        g = tr.get("g")
        if g is None or len(g) == 0:
            continue
        with_g += 1
        g_last = int(g[-1])
        t_last = _last_observed_game_time(tr, sample_hz)
        if t0 is None or t1 is None or np.isnan(t_last):
            if g_last == 1:
                survivors += 1
            continue
        present_at_stop = t_last >= (t1 - grace_s)
        if present_at_stop and g_last == 1:
            survivors += 1
    return survivors, with_g


def _run_window(data):
    trial = None
    trials = data.get("trials", [])
    if isinstance(trials, list) and trials:
        runs = [t for t in trials if t.get("label") == "Run" and t.get("endGameTime", 0) > t.get("startGameTime", 0)]
        cand = runs if runs else [t for t in trials if t.get("endGameTime", 0) > t.get("startGameTime", 0)]
        if cand:
            trial = max(cand, key=lambda t: t["endGameTime"] - t["startGameTime"])
    if not trial:
        return None, None
    return float(trial["startGameTime"]), float(trial["endGameTime"])


def compute_run_metrics(traj_path: Path):
    data = _load_json(traj_path)
    sample_hz = data.get("sampleHz", DEFAULT_SAMPLE_HZ)
    if not isinstance(sample_hz, (int, float)) or sample_hz <= 0:
        sample_hz = DEFAULT_SAMPLE_HZ
    scene = data.get("scene", data.get("level", "Unknown Scene"))
    tracks = _parse_drone_tracks(data)
    t0, t1 = _run_window(data)
    total_time_s = _infer_total_time(tracks, sample_hz)
    run_time_s = float(t1 - t0) if (t0 is not None and t1 is not None) else total_time_s
    survivors, with_g = _classify_survivors(tracks, t0, t1, sample_hz)
    return dict(
        scene=scene,
        sample_hz=sample_hz,
        total_time_s=total_time_s,
        run_time_s=run_time_s,
        survivors=survivors,
        with_g=with_g,
        total_drones=len(tracks),
    )


def _jitter(tag: str, spread=0.12):
    # Random offset to avoid perfect overlap for same values between users
    h = hash(tag) % 1000
    return (h / 1000.0 - 0.5) * spread


def _line_dot_metric(ax, runs, key, ylabel, title, colors):
    """
    Draw paired dots (no_sound vs sound) per subject with a connecting line.
    """
    subjects = sorted({r["subject"] for r in runs})
    x_map = {"no_sound": 0, "sound": 1}
    for idx, subj in enumerate(subjects):
        color = colors[idx % len(colors)]
        subj_runs = {r["condition"]: r for r in runs if r["subject"] == subj}
        xs = []
        ys = []
        for cond in ("no_sound", "sound"):
            r = subj_runs.get(cond)
            if r is None or key not in r or r[key] is None:
                xs.append(np.nan)
                ys.append(np.nan)
                continue
            xs.append(x_map[cond] + _jitter(subj, 0.06))
            ys.append(r[key])
        if not np.all(np.isnan(ys)):
            ax.plot(xs, ys, marker="o", color=color, label=f"User {subj}")
    ax.set_xticks([0, 1])
    ax.set_xticklabels(["No sound", "Sound"])
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)


def _delta_bar(ax, runs, key, ylabel, title, colors):
    """
    Plot frequency of delta values (sound - no_sound) on x-axis vs user count on y-axis.
    No binning; each unique delta gets its own bar.
    """
    from collections import Counter

    subjects = sorted({r["subject"] for r in runs})
    subj_runs = {}
    for r in runs:
        subj_runs.setdefault(r["subject"], {})[r["condition"]] = r

    deltas = []
    for subj in subjects:
        conds = subj_runs.get(subj, {})
        r_sound = conds.get("sound")
        r_silent = conds.get("no_sound")
        if not r_sound or not r_silent:
            continue
        if key not in r_sound or key not in r_silent:
            continue
        if r_sound[key] is None or r_silent[key] is None:
            continue
        deltas.append(float(r_sound[key]) - float(r_silent[key]))

    if not deltas:
        ax.text(0.5, 0.5, "No paired runs", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(title)
        return

    counts = Counter(deltas)
    xs = sorted(counts.keys())
    ys = [counts[x] for x in xs]
    # Choose a width that keeps bars from overlapping while staying visible
    width = 0.3 if len(xs) > 1 else 0.6
    ax.bar(xs, ys, width=width, color="#4c8eda", edgecolor="#333333", linewidth=0.8, align="center")
    ax.axhline(0, color="k", linestyle="--", linewidth=1.0, alpha=0.7)
    ax.set_xticks(xs)
    ax.set_xticklabels([f"{x:.2f}" for x in xs], rotation=30, ha="right")
    ax.set_ylabel("Number of users")
    ax.set_xlabel(ylabel)
    ax.set_title(title)
    ax.grid(True, axis="both", alpha=0.3)


def _compute_deltas(runs, key):
    subjects = sorted({r["subject"] for r in runs})
    subj_runs = {}
    for r in runs:
        subj_runs.setdefault(r["subject"], {})[r["condition"]] = r

    deltas = []
    for subj in subjects:
        conds = subj_runs.get(subj, {})
        r_sound = conds.get("sound")
        r_silent = conds.get("no_sound")
        if not r_sound or not r_silent:
            continue
        if key not in r_sound or key not in r_silent:
            continue
        if r_sound[key] is None or r_silent[key] is None:
            continue
        deltas.append(float(r_sound[key]) - float(r_silent[key]))
    return deltas


def _delta_hist(ax, deltas, bin_width, ylabel, title):
    if not deltas:
        ax.text(0.5, 0.5, "No paired runs", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(title)
        return

    bin_w = max(1, int(round(bin_width)))
    d_min = min(deltas)
    d_max = max(deltas)
    max_abs = max(abs(d_min), abs(d_max), bin_w)
    span = bin_w * np.ceil(max_abs / bin_w)
    # Bin edges centered around multiples of bin_w with zero in the middle
    edges = np.arange(-span - bin_w * 0.5, span + bin_w * 1.5, bin_w)
    counts, edges = np.histogram(deltas, bins=edges)
    centers = edges[:-1] + bin_w / 2.0

    ax.bar(centers, counts, width=bin_w * 0.9, color="#4c8eda", edgecolor="#333333", linewidth=0.8)
    ax.axhline(0, color="k", linestyle="--", linewidth=1.0, alpha=0.7)
    median_val = float(np.median(deltas))
    ax.axvline(median_val, color="#2ca02c", linestyle="-", linewidth=1.2, alpha=0.9)
    ax.text(
        0.5,
        -0.32,
        f"Median: {median_val:.1f}",
        transform=ax.transAxes,
        ha="center",
        va="top",
        fontsize=9,
        color="#2ca02c",
    )
    ticks = np.arange(-span, span + bin_w * 0.5, bin_w, dtype=int)
    ax.set_xticks(ticks)
    ax.set_xticklabels(ticks, rotation=30, ha="right")
    ax.set_xlim(-span - bin_w * 0.6, span + bin_w * 0.6)
    ax.set_ylabel("Number of users")
    ax.set_xlabel(ylabel)
    ax.set_title(title)
    ax.grid(True, axis="both", alpha=0.3)


def _collect_runs():
    target_batch = _target_batch_prefix()
    runs = []
    for folder in sorted(BASE_DIR.iterdir()):
        if not folder.is_dir():
            continue
        name = folder.name
        if name.endswith("_no_sound"):
            condition = "no_sound"
            subject = name[: -len("_no_sound")]
        elif name.endswith("_sound"):
            condition = "sound"
            subject = name[: -len("_sound")]
        else:
            continue

        # Parse batch prefix (before first underscore) if present
        parts = subject.split("_", 1)
        batch_prefix = parts[0] if parts else ""
        subj_id = parts[1] if len(parts) > 1 else parts[0]
        subject_label = subj_id
        if INCLUDE_ALL_BATCH and batch_prefix:
            subject_label = f"{batch_prefix}_{subj_id}"

        if (not INCLUDE_ALL_BATCH) and target_batch is not None and batch_prefix != target_batch:
            continue

        stars_path = folder / "stars.json"
        star_count = 0
        if stars_path.exists():
            try:
                star_data = _load_json(stars_path)
                star_count = len(star_data.get("records", []))
            except Exception:
                print("Error counting stars")
                star_count = 0
        traj_candidates = [p for p in folder.glob("*.json") if p.name != "stars.json"]
        if not traj_candidates:
            print(f"[WARN] No trajectory JSON in {folder}")
            continue
        traj_path = max(traj_candidates, key=lambda p: p.stat().st_mtime)
        metrics = compute_run_metrics(traj_path)
        runs.append(
            dict(
                subject=subject_label,
                batch=batch_prefix,
                condition=condition,
                folder_name=folder.name,
                stars=star_count,
                traj_file=str(traj_path),
                stars_file=str(stars_path) if stars_path.exists() else "missing",
                lost_drones=max(0, metrics["total_drones"] - metrics["survivors"]),
                **metrics,
            )
        )
    return runs


def _subject_key_from_folder(folder_name: str):
    base = folder_name
    # Match the longer suffix first to avoid partial stripping (e.g., "_no_sound" before "_sound")
    for suffix in ("_no_sound", "_sound"):
        if base.endswith(suffix):
            base = base[: -len(suffix)]
            break
    parts = base.split("_", 1)
    if len(parts) == 2:
        batch_part, user_part = parts
        batch_id = batch_part[1:] if batch_part.startswith("B") else batch_part
        if batch_id:
            return f"{batch_id}_{user_part}"
    return base


def _load_saved_metrics():
    if not JSON_OUTPUT_PATH.exists():
        return {}
    try:
        with JSON_OUTPUT_PATH.open("r") as f:
            data = json.load(f)
        if isinstance(data, dict):
            return data
        print(f"[WARN] Metrics file at {JSON_OUTPUT_PATH} is not a dict; starting fresh.")
    except Exception as exc:
        print(f"[WARN] Could not read existing metrics at {JSON_OUTPUT_PATH}: {exc}")
    return {}


def _persist_new_metrics(runs):
    saved = _load_saved_metrics()
    added = 0

    for r in runs:
        folder_name = r.get("folder_name") or Path(r.get("traj_file", "")).parent.name
        subject_key = _subject_key_from_folder(folder_name)
        cond_key = "S" if r.get("condition") == "sound" else "NS" if r.get("condition") == "no_sound" else None
        if cond_key is None:
            continue
        subj_block = saved.setdefault(subject_key, {})
        if cond_key in subj_block:
            continue
        lost = r.get("lost_drones", max(0, r["total_drones"] - r["survivors"]))
        subj_block[cond_key] = {
            "time": float(r["run_time_s"]),
            "nb_stars": int(r["stars"]),
            "lost_drones": int(lost),
        }
        added += 1

    if added:
        JSON_OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
        with JSON_OUTPUT_PATH.open("w") as f:
            json.dump(saved, f, indent=2, sort_keys=True)
        print(f"[INFO] Saved {added} new metric entries to {JSON_OUTPUT_PATH}")
    else:
        print(f"[INFO] No new metrics to save; existing file is up to date at {JSON_OUTPUT_PATH}")


def _print_summary(runs):
    print("Found runs:")
    for r in runs:
        print(
            f"  subj={r['subject']:<4s} cond={r['condition']:<8s} run_time_s={r['run_time_s']:.2f} "
            f"stars={r['stars']} survivors={r['survivors']} lost={r.get('lost_drones', 0)} with_g={r['with_g']} drones={r['total_drones']}"
        )
    print("\nCondition averages:")
    for cond in ("no_sound", "sound"):
        subset = [r for r in runs if r["condition"] == cond]
        if not subset:
            print(f"  {cond}: no data")
            continue
        rt_mean = float(np.mean([r["run_time_s"] for r in subset]))
        stars_mean = float(np.mean([r["stars"] for r in subset]))
        surv_mean = float(np.mean([r["survivors"] for r in subset]))
        print(
            f"  {cond}: mean run_time={rt_mean:.2f}s, mean stars={stars_mean:.2f}, mean survivors={surv_mean:.2f}"
        )


def _selected_traj_path():
    """
    Return the trajectory JSON path for the configured batch/uid/condition.
    """
    if not ENABLE_TRAJECTORY_SLIDER:
        return None
    if not TRAJ_BATCH or not TRAJ_UID or not TRAJ_CONDITION:
        return None
    folder = BASE_DIR / f"{TRAJ_BATCH}_{TRAJ_UID}_{TRAJ_CONDITION}"
    if not folder.exists():
        print(f"[INFO] Selected trajectory folder not found: {folder}")
        return None
    candidates = [p for p in folder.glob("*.json") if p.name != "stars.json"]
    if not candidates:
        print(f"[INFO] No trajectory JSON found under {folder}")
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def _load_obstacle_course(path=None):
    """
    Load obstacle course export (if present).
    """
    target = path if path is not None else OBSTACLE_JSON_DEFAULT
    if not target.exists():
        print("No obstacle .json file found")
        return None
    try:
        with target.open("r") as f:
            return json.load(f)
    except Exception as exc:
        print(f"[WARN] Failed to read obstacle course JSON: {exc}")
        return None


def _load_star_records(folder: Path):
    """
    Load star pickup records (positions and times) from stars.json if present.
    """
    path = folder / "stars.json"
    if not path.exists():
        return []
    try:
        data = _load_json(path)
    except Exception:
        print(f"[WARN] Could not read stars.json in {folder}")
        return []
    records = data.get("records", [])
    stars = []
    for rec in records:
        try:
            stars.append(
                {
                    "t": float(rec.get("t", 0.0)),
                    "x": float(rec.get("x", 0.0)),
                    "z": float(rec.get("z", 0.0)),
                }
            )
        except Exception:
            continue
    return stars


def _time_array_for(track, sample_hz):
    if track.get("t") is not None and len(track["t"]) > 0:
        return np.asarray(track["t"], dtype=float)
    n = len(track.get("x", []))
    return np.arange(n, dtype=float) / float(sample_hz)


def _clip_to_time(xs, zs, ts, T):
    """Return x,z arrays truncated to last index with t <= T (inclusive)."""
    if ts is None or len(ts) == 0:
        return np.array([]), np.array([])
    idx = np.searchsorted(ts, T, side="right") - 1
    if idx < 0:
        return np.array([]), np.array([])
    return xs[: idx + 1], zs[: idx + 1]


def _plot_selected_trajectory_slider():
    """
    Create an interactive window showing the configured trajectory with a time slider.
    """
    traj_path = _selected_traj_path()
    if traj_path is None:
        return
    try:
        data = _load_json(traj_path)
    except Exception:
        print(f"[WARN] Could not read trajectory file: {traj_path}")
        return
    star_records = _load_star_records(traj_path.parent)

    sample_hz = data.get("sampleHz", DEFAULT_SAMPLE_HZ)
    if not isinstance(sample_hz, (int, float)) or sample_hz <= 0:
        sample_hz = DEFAULT_SAMPLE_HZ

    tracks = _parse_drone_tracks(data)
    drone_tracks = {
        tr["name"]: tr
        for tr in tracks
        if len(tr.get("x", [])) > 0 and len(tr.get("z", [])) > 0
    }
    if not drone_tracks:
        print("[INFO] No drone tracks available for slider plot.")
        return

    per_drone_times = {name: _time_array_for(tr, sample_hz) for name, tr in drone_tracks.items()}
    global_t_min = min(ts[0] for ts in per_drone_times.values() if len(ts) > 0)
    global_t_max = max(ts[-1] for ts in per_drone_times.values() if len(ts) > 0)

    t0_run, t1_run = _run_window(data)
    slider_t0 = t0_run if t0_run is not None else global_t_min
    slider_t1 = t1_run if t1_run is not None else global_t_max
    if slider_t1 < slider_t0:
        slider_t0, slider_t1 = global_t_min, global_t_max

    from matplotlib.widgets import Slider, Button

    fig, ax = plt.subplots(figsize=(8, 8))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    plt.subplots_adjust(left=0.08, right=0.98, top=0.92, bottom=0.16)

    try:
        cmap = plt.get_cmap("tab20")
        names_sorted = sorted(drone_tracks.keys())
        N = getattr(cmap, "N", 20)
        name_to_color = {n: cmap(i % N) for i, n in enumerate(names_sorted)}
    except Exception:
        name_to_color = {}

    drone_lines = {}
    drone_markers = {}
    for name, tr in drone_tracks.items():
        col = name_to_color.get(name, None)
        line, = ax.plot([], [], linewidth=2.2, alpha=0.9, label=name, color=col)
        mark = ax.scatter([], [], s=36, marker="o", color=col if col else None, zorder=3)
        drone_lines[name] = line
        drone_markers[name] = mark

    centroid_marker = ax.scatter([], [], s=64, marker="x", color="k", zorder=4, label="Centroid @T")

    # Track extents for zoom slider bounds
    xs_all = np.concatenate([np.asarray(tr["x"], dtype=float) for tr in drone_tracks.values()])
    zs_all = np.concatenate([np.asarray(tr["z"], dtype=float) for tr in drone_tracks.values()])
    width_min = 5.0
    width_max = max(width_min * 2, float(np.max([np.ptp(xs_all), np.ptp(zs_all)])) * 2.0 if len(xs_all) else 100.0)
    width_state = {"w": float(np.clip(CORRIDOR_WIDTH, width_min, width_max))}

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title(
        f"Trajectory slider: {TRAJ_BATCH}_{TRAJ_UID}_{TRAJ_CONDITION}\nFile: {traj_path.name}"
    )
    ax.grid(True, alpha=0.25)
    ax.set_xlim(-width_state["w"] / 2.0, width_state["w"] / 2.0)
    ax.set_ylim(-width_state["w"] * 0.75, width_state["w"] * 0.75)

    obstacle_path = _obstacle_json_path_for_batch(TRAJ_BATCH)
    obstacles = _load_obstacle_course(obstacle_path)
    obstacle_patches = []

    def _add_wall_patch(wall, color="#000000", alpha=1):
        if not wall:
            return
        x = wall.get("x", None)
        z = wall.get("z", None)
        w = wall.get("width", None)
        l = wall.get("length", None)
        rot = wall.get("rotationY", 0.0)
        if None in (x, z, w, l):
            return
        rect = Rectangle(
            (x - w * 0.5, z - l * 0.5),
            w,
            l,
            facecolor=color,
            edgecolor="#000000",
            alpha=alpha,
            linewidth=0.8,
            zorder=0,
        )
        if rot:
            rotation = transforms.Affine2D().rotate_deg_around(x, z, -float(rot))
            rect.set_transform(rotation + ax.transData)
        ax.add_patch(rect)
        obstacle_patches.append(rect)

    if obstacles:
        for bw in obstacles.get("boundaryWalls", []):
            _add_wall_patch(bw, color="#000000", alpha=1)
        for gap in obstacles.get("gaps", []):
            _add_wall_patch(gap.get("left"), color="#000000", alpha=1)
            _add_wall_patch(gap.get("right"), color="#000000", alpha=1)

    if star_records:
        xs = [s["x"] for s in star_records]
        zs = [s["z"] for s in star_records]
        ax.scatter(
            xs,
            zs,
            s=52,
            marker="*",
            color="#f5c400",
            edgecolors="k",
            linewidths=0.7,
            zorder=6,
            label="Stars",
        )

    ax_T = plt.axes([0.14, 0.06, 0.74, 0.04])
    slider = Slider(ax=ax_T, label="Time (s)", valmin=slider_t0, valmax=slider_t1, valinit=slider_t0)

    ax_zoom = plt.axes([0.14, 0.02, 0.74, 0.03])
    zoom_slider = Slider(ax=ax_zoom, label="Zoom", valmin=width_min, valmax=width_max, valinit=width_state["w"])

    def _set_view(cx, cz):
        """Center the view on centroid and use fixed corridor extents."""
        half_w = width_state["w"] / 2.0
        half_h = width_state["w"] * 0.75
        ax.set_xlim(cx - half_w, cx + half_w)
        ax.set_ylim(cz - half_h, cz + half_h)

    def _update_plot(T):
        centroid_pts = []
        for name, tr in drone_tracks.items():
            xs = np.asarray(tr["x"], dtype=float)
            zs = np.asarray(tr["z"], dtype=float)
            ts = per_drone_times[name]
            xseg, zseg = _clip_to_time(xs, zs, ts, T)
            drone_lines[name].set_data(xseg, zseg)
            if len(xseg) > 0:
                drone_markers[name].set_offsets(np.c_[xseg[-1], zseg[-1]])
                centroid_pts.append((xseg[-1], zseg[-1]))
            else:
                drone_markers[name].set_offsets(np.c_[[], []])

        if centroid_pts:
            cx, cz = np.mean(np.asarray(centroid_pts), axis=0)
            centroid_marker.set_offsets(np.c_[cx, cz])
            _set_view(cx, cz)
        else:
            centroid_marker.set_offsets(np.c_[[], []])

        fig.canvas.draw_idle()

    def on_zoom(val):
        width_state["w"] = float(np.clip(val, width_min, width_max))
        _update_plot(slider.val)

    zoom_slider.on_changed(on_zoom)

    _update_plot(slider.val)

    def on_slider(val):
        _update_plot(val)

    slider.on_changed(on_slider)

    # --- Play/Pause button ---
    ax_play = plt.axes([0.05, 0.06, 0.06, 0.04])
    btn_play = Button(ax_play, "▶", hovercolor="#d0d0d0")
    playing = {"state": False}
    timer = fig.canvas.new_timer(interval=30)

    def _step_frame():
        if not playing["state"]:
            return
        cur = slider.val
        new = cur + max((slider_t1 - slider_t0) / 400.0, 0.02)
        if new > slider_t1:
            new = slider_t0
        slider.set_val(new)

    timer.add_callback(_step_frame)

    def on_play(event):
        playing["state"] = not playing["state"]
        btn_play.label.set_text("⏸" if playing["state"] else "▶")
        if playing["state"]:
            timer.start()
        else:
            timer.stop()
        fig.canvas.draw_idle()

    btn_play.on_clicked(on_play)

    # Keep references so widgets stay responsive
    _SLIDER_FIGS.append((fig, slider, zoom_slider, btn_play))
    ax.legend(loc="center left", bbox_to_anchor=(-0.22, 0.5), fontsize=8)
    print(f"[INFO] Loaded trajectory for slider from {traj_path}")

def main():
    runs = _collect_runs()

    if not runs:
        raise SystemExit("No runs found under Assets/Data/default/Trajectories/")
    _persist_new_metrics(runs)
    _print_summary(runs)

    colors = plt.get_cmap("tab10").colors

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    _line_dot_metric(axes[0], runs, "run_time_s", "Run time (s)", "Run time by condition", colors)
    _line_dot_metric(axes[1], runs, "stars", "Stars collected", "Stars collected by condition", colors)
    _line_dot_metric(axes[2], runs, "survivors", "Survivor drones", "Survivors by condition", colors)
    plt.suptitle("Audio vs no-audio comparison across subjects")
    # Put legend below plots
    handles, labels = axes[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper center", ncol=min(len(handles), 5), bbox_to_anchor=(0.5, 0.02))
    plt.tight_layout(rect=[0, 0.06, 1, 0.94])
    plt.savefig(OUT_PNG, dpi=150)
    print(f"Saved plot to {OUT_PNG}")

    # --- Composite score figure ---
    for r in runs:
        lost = r.get("lost_drones", max(0, r["total_drones"] - r["survivors"]))
        r["score"] = SCORE_TIME_W * r["run_time_s"] + SCORE_STARS_W * r["stars"] + SCORE_LOST_W * lost
    # Delta histogram for composite score (fixed bin width)
    score_deltas = _compute_deltas(runs, "score")
    fig_score, ax_score = plt.subplots(1, 1, figsize=(6, 5))
    _delta_hist(ax_score, score_deltas, 100, "Δ composite score", "Composite score delta (sound - no sound)")
    fig_score.tight_layout()

    fig_delta, delta_axes = plt.subplots(1, 3, figsize=(14, 7.0))
    keys = [
        ("run_time_s", "Δ run time (s)", "Run time delta"),
        ("stars", "Δ stars", "Stars collected delta"),
        ("survivors", "Δ survivors", "Survivors delta"),
    ]
    deltas_map = {k: _compute_deltas(runs, k) for k, _, _ in keys}

    from matplotlib.widgets import Slider

    fig_delta.subplots_adjust(bottom=0.34, top=0.88, wspace=0.32)
    slider_positions = [
        [0.12, 0.10, 0.22, 0.04],
        [0.40, 0.10, 0.22, 0.04],
        [0.68, 0.10, 0.22, 0.04],
    ]

    sliders = []
    slider_texts = []
    for ax_delta, (k, ylabel, title), pos in zip(delta_axes, keys, slider_positions):
        s_ax = fig_delta.add_axes(pos)
        init_v, min_v, max_v = DELTA_BIN_LIMITS.get(k, (DELTA_BIN_INIT, DELTA_BIN_MIN, DELTA_BIN_MAX))
        slider = Slider(
            ax=s_ax,
            label="",
            valmin=min_v,
            valmax=max_v,
            valinit=init_v,
            valstep=1,
        )
        slider.valtext.set_visible(False)
        val_txt = s_ax.text(0.5, -0.8, f"{init_v}", transform=s_ax.transAxes, ha="center", va="top", fontsize=8)

        def _mk_cb(ax_ref, deltas_ref, ylabel_ref, title_ref):
            def _cb(val):
                ax_ref.cla()
                _delta_hist(ax_ref, deltas_ref, val, ylabel_ref, title_ref)
                fig_delta.canvas.draw_idle()
            return _cb

        cb = _mk_cb(ax_delta, deltas_map[k], ylabel, title)
        slider.on_changed(cb)
        cb(init_v)
        sliders.append(slider)
        slider_texts.append(val_txt)

        def _mk_txt(txt_ref):
            def _txt_cb(val):
                txt_ref.set_text(f"{int(val)}")
            return _txt_cb

        slider.on_changed(_mk_txt(val_txt))

    fig_delta.suptitle("Per-user delta histogram (sound - no sound)")

    _plot_selected_trajectory_slider()
    plt.show()


if __name__ == "__main__":
    main()
