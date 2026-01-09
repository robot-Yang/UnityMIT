import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import pandas as pd

try:
    import tkinter as tk
    from tkinter import filedialog
except Exception:
    tk = None

PLOT_TRAJ_SCRIPT = Path(__file__).with_name("plot_traj_13.py")
DEFAULT_DATA_DIR = Path("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default")
RUNS_CACHE = Path(__file__).with_name("plot_all4_runs.json")
OUT_DIR = Path(__file__).with_name("outputs")
OUT_DIR.mkdir(parents=True, exist_ok=True)

def normalize_condition(text: str):
    t = (text or "").strip().lower()
    if "without" in t or t == "no":
        return "Without Haptic"
    if "with" in t or t == "yes" or t == "haptic":
        return "With Haptic"
    return None

def canonical_pid(pid: str) -> str:
    """Return a normalized participant id (strip spaces, drop leading 'P')."""
    s = str(pid).strip()
    if s.lower().startswith("p"):
        s = s[1:]
    return s

def display_pid(pid: str) -> str:
    """Return display label 'P#' based on canonical id."""
    base = canonical_pid(pid)
    return f"P{base}" if base else str(pid)

def ask_for_json_file(title: str) -> Optional[Path]:
    """Pick a JSON file via file dialog if available, otherwise ask for a path."""
    if tk is not None:
        try:
            root = tk.Tk()
            # Place near top-left and keep on top so the dialog is visible
            root.geometry("+80+80")
            root.update_idletasks()
            root.lift()
            root.attributes("-topmost", True)
            path = filedialog.askopenfilename(
                parent=root,
                initialdir=str(DEFAULT_DATA_DIR),
                filetypes=[("JSON", "*.json")],
                title=title,
            )
            root.attributes("-topmost", False)
            root.destroy()
            if path:
                return Path(path)
        except Exception:
            pass
    resp = input(f"{title}\nEnter JSON path (blank to skip): ").strip()
    return Path(resp) if resp else None

def gather_runs():
    def select_runs_for_condition(participant: str, condition: str, expected: int = 2):
        picked = []
        print(f"\nSelect up to {expected} runs for {participant} — {condition} (blank to stop).")
        for i in range(expected):
            path = ask_for_json_file(f"{participant} ({condition}) run {i+1}")
            if not path:
                break
            picked.append(path)
        return picked

    runs = []
    print("Enter each participant, then pick two runs per condition (With/Without haptic).")
    print("Leave participant blank to finish.")
    while True:
        participant = input("\nParticipant ID: ").strip()
        if participant == "":
            break

        for cond_label in ["Without Haptic", "With Haptic"]:
            resp = input(f"Add runs for {cond_label}? [Y/n]: ").strip().lower()
            if resp.startswith("n"):
                continue
            paths = select_runs_for_condition(participant, cond_label, expected=2)
            for pth in paths:
                runs.append(dict(participant=participant, condition=cond_label, path=pth))

    return runs

def save_runs(runs):
    """Persist the selected runs so they can be reused next time."""
    serializable = []
    for r in runs:
        serializable.append({
            "participant": r["participant"],
            "condition": r["condition"],
            "path": str(r["path"]),
        })
    try:
        RUNS_CACHE.write_text(json.dumps(serializable, indent=2))
        print(f"\nSaved selection to {RUNS_CACHE}")
    except Exception as e:
        print(f"\nCould not save selection ({e})")

def load_saved_runs():
    """Return cached runs if the cache file exists."""
    if not RUNS_CACHE.exists():
        return []
    try:
        data = json.loads(RUNS_CACHE.read_text())
        runs = []
        if isinstance(data, list):
            for item in data:
                if not isinstance(item, dict):
                    continue
                p = item.get("participant"); c = item.get("condition"); path = item.get("path")
                if not (p and c and path):
                    continue
                runs.append({"participant": p, "condition": c, "path": Path(path)})
        return runs
    except Exception as e:
        print(f"Warning: could not load cached runs ({e}); ignoring cache.")
        return []

def maybe_use_saved_runs():
    saved = load_saved_runs()
    if not saved:
        return None
    print("\nFound saved file selections:")
    for i, r in enumerate(saved, start=1):
        exists = "OK" if Path(r["path"]).exists() else "MISSING"
        print(f"  {i}. {r['participant']} — {r['condition']} — {r['path']} [{exists}]")
    resp = input("Use saved selection? [Y/n]: ").strip().lower()
    if resp.startswith("n"):
        return None
    return saved

def fetch_metrics(json_path: Path):
    cmd = [sys.executable, str(PLOT_TRAJ_SCRIPT), "--metrics-only", "--input", str(json_path)]
    env = os.environ.copy()
    env.setdefault("MPLBACKEND", "Agg")
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, cwd=PLOT_TRAJ_SCRIPT.parent, env=env, timeout=90)
    except subprocess.TimeoutExpired:
        print(f"  Timed out processing {json_path}")
        return None
    if result.returncode != 0:
        print(f"  Failed to process {json_path}:\n{result.stderr}")
        return None
    try:
        return json.loads(result.stdout)
    except json.JSONDecodeError:
        print(f"  Could not decode metrics output for {json_path}.\nRaw output:\n{result.stdout}")
        return None

# Gather runs (reuse cache if desired) and compute metrics on the fly via plot_traj_13.py
saved = maybe_use_saved_runs()
runs = []
if saved is not None:
    runs = saved
    add_more = input("Add new participants/runs on top of saved selection? [y/N]: ").strip().lower()
    if add_more.startswith("y"):
        runs.extend(gather_runs())
else:
    runs = gather_runs()

if runs:
    resp = input("Save this selection for next time? [Y/n]: ").strip().lower()
    if not resp.startswith("n"):
        save_runs(runs)

if not runs:
    print("No runs entered; nothing to plot.")
    sys.exit(0)

records = []
for run in runs:
    json_path = Path(run["path"])
    if not json_path.exists():
        print(f"  Skipping missing file: {json_path}")
        continue
    metrics = fetch_metrics(json_path)
    if not metrics:
        continue
    participant_label = display_pid(run["participant"])
    records.append({
        "Participant": participant_label,
        "Condition": run["condition"],
        "Time_s": metrics.get("run_total_spent_time_s", float("nan")),
        "AvgDist_m": metrics.get("avg_centroid_ref_dist_m", float("nan")),
        "Survived": metrics.get("survivors", float("nan")),
        "Disconnected": metrics.get("disconnected", float("nan")),
        "Crashed": metrics.get("crashed_total", float("nan")),
        "Split": metrics.get("split_metric", float("nan")),
        "Sweep_pct": metrics.get("overall_sweep_pct", float("nan")),
    })

if not records:
    print("No valid metrics gathered; aborting plot.")
    sys.exit(1)

perf = pd.DataFrame(records)
perf_grp = perf.groupby(["Participant", "Condition"]).mean(numeric_only=True).reset_index()

# ===== Questionnaire data (still manual for now) =====
subj_data = [
    ["P1", "Preference", 3, 4],
    ["P1", "Mentally Draining", 2, 2],
    ["P1", "Swarm Awareness", 3, 4],
    ["P1", "Environment Awareness", 2, 3],
    ["P2", "Preference", 2, 4],
    ["P2", "Mentally Draining", 2, 4],
    ["P2", "Swarm Awareness", 1, 4],
    ["P2", "Environment Awareness", 2, 3],
    ["P3", "Preference", 4, 4],
    ["P3", "Mentally Draining", 2, 3],
    ["P3", "Swarm Awareness", 1, 3],
    ["P3", "Environment Awareness", 1, 2],
    ["P4", "Preference", 1, 4],
    ["P4", "Mentally Draining", 2, 3],
    ["P4", "Swarm Awareness", 2, 4],
    ["P4", "Environment Awareness", 3, 4],
    ["P5", "Preference", 3, 4],
    ["P5", "Mentally Draining", 3, 4],
    ["P5", "Swarm Awareness", 1, 4],
    ["P5", "Environment Awareness", 4, 5],
]
subj = pd.DataFrame(subj_data, columns=["Participant", "Metric", "Without", "With"])
subj["Participant"] = subj["Participant"].apply(display_pid)

participants_obj = sorted(perf_grp["Participant"].unique())
participants_subj = sorted(subj["Participant"].unique())
all_participants = sorted(set(participants_obj) | set(participants_subj))
cmap = plt.get_cmap("tab10")
colors = {p: cmap(i % cmap.N) for i, p in enumerate(all_participants)}

_label_offsets_pts = [8, -8, 14, -14, 20, -20, 26, -26]
_label_offsets_x = [10, -10, 18, -18, 24, -24, 30, -30]
def label_offset_points(pid: str) -> int:
    """Return a small vertical pixel offset for annotations to reduce overlap."""
    try:
        idx = all_participants.index(pid)
    except ValueError:
        idx = 0
    return _label_offsets_pts[idx % len(_label_offsets_pts)]

def annotate_identical_series(ax, series_pairs, colors, round_decimals: int = 3):
    """
    Annotate only participants whose entire 2-point series (Without/With) is identical.
    series_pairs: dict pid -> (without_val, with_val)   (values can be None/NaN)
    """
    buckets = {}
    for pid, pair in series_pairs.items():
        w0, w1 = pair
        if pd.isna(w0) or pd.isna(w1):
            continue
        key = (round(float(w0), round_decimals), round(float(w1), round_decimals))
        buckets.setdefault(key, []).append(pid)
    for pids in buckets.values():
        if len(pids) < 2:
            continue
        for i, pid in enumerate(pids):
            dy = 0  # keep same vertical position as the marker
            dx = _label_offsets_x[i % len(_label_offsets_x)]
            w0, w1 = series_pairs[pid]
            ax.annotate(pid, ("Without Haptic", w0),
                        xytext=(dx, dy), textcoords="offset points",
                        ha="center", va="bottom", fontsize=8,
                        color=colors[pid], alpha=0.9)
            ax.annotate(pid, ("With Haptic", w1),
                        xytext=(dx, dy), textcoords="offset points",
                        ha="center", va="bottom", fontsize=8,
                        color=colors[pid], alpha=0.9)

# ===== FIGURE 1: Objective performance =====
obj_metrics = ["Time_s", "AvgDist_m", "Disconnected", "Crashed", "Split", "Sweep_pct"]
obj_titles  = ["Completion Time (s)",
               "Centroid→Reference Distance (m)",
               "Disconnected Drones",
               "Crashed Drones",
               "Split Metric",
               "Area Coverage Rate (%)"]

ncols = 3
nrows = (len(obj_metrics) + ncols - 1) // ncols
fig, axes = plt.subplots(nrows, ncols, figsize=(13, 8))
fig.suptitle("Objective Performance: Without vs With Haptic", fontsize=14, fontweight="bold")

for ax, metric, title in zip(axes.flat, obj_metrics, obj_titles):
    series_pairs = {}
    seen_series = {}
    for p in participants_obj:
        sub = perf_grp[perf_grp["Participant"] == p].copy()
        sub["Condition"] = pd.Categorical(
            sub["Condition"],
            categories=["Without Haptic", "With Haptic"],
            ordered=True
        )
        sub = sub.sort_values("Condition")
        vals = sub.set_index("Condition")[metric].to_dict()
        pair = (vals.get("Without Haptic", float("nan")), vals.get("With Haptic", float("nan")))
        linestyle = "--" if pair in seen_series else "-"
        seen_series[pair] = seen_series.get(pair, 0) + 1
        ax.plot(sub["Condition"], sub[metric],
                marker="o", linewidth=2, color=colors[p], label=p, linestyle=linestyle)
        series_pairs[p] = pair

    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.set_facecolor("#f9f9f9")
    ax.set_ylabel(title)
    annotate_identical_series(ax, series_pairs, colors)

for extra_ax in list(axes.flat)[len(obj_metrics):]:
    extra_ax.axis("off")

first_ax = axes.flat[0]
handles, labels = first_ax.get_legend_handles_labels()
fig.legend(handles, labels, loc="lower center", ncol=min(5, len(all_participants)), frameon=False)
plt.tight_layout(rect=[0, 0.08, 1, 0.94])
objective_path = OUT_DIR / "plot_all4_objective.png"
fig.savefig(objective_path, dpi=150)

# ===== FIGURE 1b: Objective boxplots =====
fig_box, axes_box = plt.subplots(nrows, ncols, figsize=(13, 8))
fig_box.suptitle("Objective Performance Boxplots: Without vs With Haptic", fontsize=14, fontweight="bold")
for ax, metric, title in zip(axes_box.flat, obj_metrics, obj_titles):
    sub_df = perf.pivot_table(index="Participant", columns="Condition", values=metric)
    sub_df = sub_df[["Without Haptic", "With Haptic"]] if "Without Haptic" in sub_df.columns and "With Haptic" in sub_df.columns else sub_df
    sub_df.boxplot(ax=ax, widths=0.5, patch_artist=True,
                   boxprops=dict(facecolor="#dddddd", color="#444444"),
                   medianprops=dict(color="black"))
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.set_facecolor("#fafafa")

for extra_ax in list(axes_box.flat)[len(obj_metrics):]:
    extra_ax.axis("off")

plt.tight_layout(rect=[0, 0.05, 1, 0.95])
objective_box_path = OUT_DIR / "plot_all4_objective_boxplot.png"
fig_box.savefig(objective_box_path, dpi=150)

# ===== FIGURE 2: Questionnaire results in 2x2 =====
fig_q, axes_q = plt.subplots(2, 2, figsize=(11, 6))
fig_q.suptitle("Questionnaire Results: Without vs With Haptic (P1–P4)", fontsize=14, fontweight="bold")

q_metrics = ["Preference", "Mentally Draining", "Swarm Awareness", "Environment Awareness"]
q_titles = {
    "Preference": "Liking",
    "Mentally Draining": "Mentally Draining",
    "Swarm Awareness": "Swarm Awareness",
    "Environment Awareness": "Environment Awareness",
}

for ax, metric in zip(axes_q.flat, q_metrics):
    series_pairs = {}
    seen_series = {}
    for p in participants_subj:
        row = subj[(subj["Participant"] == p) & (subj["Metric"] == metric)].iloc[0]
        x = ["Without Haptic", "With Haptic"]
        y = [row["Without"], row["With"]]
        pair = (y[0], y[1])
        linestyle = "--" if pair in seen_series else "-"
        seen_series[pair] = seen_series.get(pair, 0) + 1
        ax.plot(x, y, marker="o", linewidth=2, color=colors[p], label=p, alpha=1.0, linestyle=linestyle)
        series_pairs[p] = (y[0], y[1])
    ax.set_title(q_titles.get(metric, metric))
    ax.set_ylim(0.5, 5.5)
    ax.set_ylabel("Score (1–5)")
    ax.grid(True, alpha=0.3)
    ax.set_facecolor("#f9f9f9")
    annotate_identical_series(ax, series_pairs, colors, round_decimals=2)

handles_q, labels_q = axes_q[0, 0].get_legend_handles_labels()
fig_q.legend(handles_q, labels_q, loc="lower center", ncol=max(len(participants_subj), 1), frameon=False)
plt.tight_layout(rect=[0, 0.12, 1, 0.94])
questionnaire_path = OUT_DIR / "plot_all4_questionnaire.png"
fig_q.savefig(questionnaire_path, dpi=150)

# ===== FIGURE 2b: Questionnaire boxplots =====
fig_qb, axes_qb = plt.subplots(2, 2, figsize=(11, 6))
fig_qb.suptitle("Questionnaire Boxplots: Without vs With Haptic", fontsize=14, fontweight="bold")
for ax, metric in zip(axes_qb.flat, q_metrics):
    metric_rows = subj[subj["Metric"] == metric]
    data = pd.DataFrame({
        "Without Haptic": metric_rows["Without"],
        "With Haptic": metric_rows["With"]
    })
    data.boxplot(ax=ax, widths=0.5, patch_artist=True,
                 boxprops=dict(facecolor="#dddddd", color="#444444"),
                 medianprops=dict(color="black"))
    ax.set_title(metric)
    ax.set_ylim(0.5, 5.5)
    ax.set_ylabel("Score (1–5)")
    ax.grid(alpha=0.3)
    ax.set_facecolor("#fafafa")

plt.tight_layout(rect=[0, 0.05, 1, 0.95])
questionnaire_box_path = OUT_DIR / "plot_all4_questionnaire_boxplot.png"
fig_qb.savefig(questionnaire_box_path, dpi=150)

print(f"\nSaved objective plot to {objective_path}")
print(f"Saved objective boxplot to {objective_box_path}")
print(f"Saved questionnaire plot to {questionnaire_path}")
print(f"Saved questionnaire boxplot to {questionnaire_box_path}")
plt.show()
