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
import numpy as np

BASE_DIR = Path("Assets/Data/default/Trajectories")
OUT_DIR = Path("outputs")
OUT_DIR.mkdir(parents=True, exist_ok=True)
OUT_PNG = OUT_DIR / "audio_vs_no_audio_metrics.png"

# Batch filtering: set INCLUDE_ALL_BATCH = False and BATCH_ID = "B3" (example)
# to only include folders starting with that batch prefix (before the first underscore).
INCLUDE_ALL_BATCH = True
BATCH_ID = "B1"

SCENE_SWITCH_GRACE_S = 1.0
DEFAULT_SAMPLE_HZ = 5.0


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
            g_arr = [fr.get("g", None) for fr in frames]
            tracks.append(
                dict(
                    name=name,
                    id=traj.get("id", None),
                    t=np.array(t_arr, dtype=float) if (t_arr and t_arr[0] is not None) else None,
                    x=np.array(x_arr, dtype=float),
                    g=np.array(g_arr, dtype=float) if any(v is not None for v in g_arr) else None,
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
            g_arr = [p.get("g", None) for p in pos]
            t_here = top_time if (top_time is not None and len(top_time) == len(x_arr)) else None
            tracks.append(
                dict(
                    name=name,
                    id=entry.get("droneId", None),
                    t=t_here,
                    x=np.array(x_arr, dtype=float),
                    g=np.array(g_arr, dtype=float)
                    if any(v is not None for v in g_arr)
                    else None,
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


def _collect_runs():
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

        if (not INCLUDE_ALL_BATCH) and batch_prefix != BATCH_ID:
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
                subject=subj_id,
                batch=batch_prefix,
                condition=condition,
                stars=star_count,
                traj_file=str(traj_path),
                stars_file=str(stars_path) if stars_path.exists() else "missing",
                **metrics,
            )
        )
    return runs


def _print_summary(runs):
    print("Found runs:")
    for r in runs:
        print(
            f"  subj={r['subject']:<4s} cond={r['condition']:<8s} run_time_s={r['run_time_s']:.2f} "
            f"stars={r['stars']} survivors={r['survivors']} with_g={r['with_g']} drones={r['total_drones']}"
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

def main():
    runs = _collect_runs()

    if not runs:
        raise SystemExit("No runs found under Assets/Data/default/Trajectories/")
    _print_summary(runs)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    colors = plt.get_cmap("tab10").colors
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
    plt.show()


if __name__ == "__main__":
    main()
