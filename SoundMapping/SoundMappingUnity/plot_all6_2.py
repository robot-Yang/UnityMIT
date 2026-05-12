import json
import os
import re
import subprocess
import sys
from pathlib import Path
from datetime import datetime, timezone
from typing import Optional

import matplotlib.pyplot as plt
from matplotlib.colors import PowerNorm
import pandas as pd
import numpy as np
from itertools import combinations

try:
    from scipy.stats import mannwhitneyu, kruskal, wilcoxon
except Exception:
    mannwhitneyu = None
    kruskal = None
    wilcoxon = None

try:
    import tkinter as tk
    from tkinter import filedialog, messagebox
except Exception:
    tk = None
    messagebox = None

PLOT_TRAJ_SCRIPT = Path(__file__).with_name("plot_traj_15.py")
DEFAULT_DATA_DIR = Path("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default")
RUNS_CACHE = Path(__file__).with_name("plot_all6_runs.json")
METRICS_CACHE = Path(__file__).with_name("plot_all6_metrics.json")
SELECTION_CACHE = Path(__file__).with_name("plot_all6_selection.json")
OUT_DIR = Path(__file__).with_name("outputs")
OUT_DIR.mkdir(parents=True, exist_ok=True)
CONDITIONS = ["Without Haptic", "Mini Map", "With Haptic"]
CONDITION_DISPLAY = {
    "Without Haptic": "FPV only",
    "Mini Map": "FPV + minimap",
    "With Haptic": "FPV + haptic",
}
SHOW_COUNTS_ON_FIGURES = True
SHOW_QUESTIONNAIRE_PLOTS = False
ENABLE_POST_GUI_EXCLUSION = False
GAZE_BINS = 50
# Smaller gamma brightens low-density regions in shared heatmaps.
GAZE_DENSITY_GAMMA = 0.45
OUT_GAZE_GROUP_HEATMAP = OUT_DIR / "plot_all6_gaze_group_heatmaps.png"
OUT_GAZE_METRICS_BOXPLOT = OUT_DIR / "plot_all6_gaze_metrics_boxplot.png"
OUT_LEARNING_CURVE_BOXPLOT = OUT_DIR / "plot_all6_learning_curve_boxplot.png"

def normalize_condition(text: str):
    t = (text or "").strip().lower()
    if "without" in t or t == "no":
        return "Without Haptic"
    if "with" in t or t == "yes" or t == "haptic":
        return "With Haptic"
    return None

def display_condition(cond: str) -> str:
    return CONDITION_DISPLAY.get(cond, cond)

def condition_label(cond: str, count=None, noun: str = "n") -> str:
    base = display_condition(cond)
    if not SHOW_COUNTS_ON_FIGURES or count is None:
        return base
    return f"{base} ({noun}={int(count)})"

def prompt_yes_no(prompt: str, default: bool = True) -> bool:
    suffix = " [Y/n]: " if default else " [y/N]: "
    ans = input(prompt + suffix).strip().lower()
    if not ans:
        return default
    return ans in ("y", "yes")

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

def normalize_path(path: Path) -> Path:
    p = Path(path).expanduser()
    try:
        return p.resolve()
    except Exception:
        return p.absolute()

def has_duplicate_path(path: Path, runs) -> bool:
    key = str(normalize_path(path))
    for r in runs:
        rp = r.get("path")
        if rp is None:
            continue
        if str(normalize_path(Path(rp))) == key:
            return True
    return False

def dedupe_runs(runs):
    """Remove duplicate file paths while preserving first occurrence."""
    out = []
    seen = set()
    dup_count = 0
    for r in runs:
        rp = r.get("path")
        if rp is None:
            continue
        key = str(normalize_path(Path(rp)))
        if key in seen:
            dup_count += 1
            print(f"  Duplicate removed: {rp}")
            continue
        seen.add(key)
        out.append(r)
    if dup_count > 0:
        print(f"Removed {dup_count} duplicate run selections.")
    return out

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

def select_runs_for_condition(participant: str, condition: str, expected: int = 2, existing_runs=None):
    picked = []
    existing_runs = list(existing_runs) if existing_runs else []
    print(f"\nSelect up to {expected} runs for {participant} — {display_condition(condition)} (blank to stop).")
    while len(picked) < expected:
        run_idx = len(picked) + 1
        path = ask_for_json_file(f"{participant} ({condition}) run {run_idx}")
        if not path:
            break
        if has_duplicate_path(path, existing_runs) or any(str(normalize_path(path)) == str(normalize_path(p)) for p in picked):
            print(f"  Duplicate file ignored: {path}")
            continue
        picked.append(path)
    return picked

def add_runs_for_existing_participants(participants, conditions, existing_runs=None):
    runs = []
    existing_runs = list(existing_runs) if existing_runs else []
    for participant in participants:
        for cond_label in conditions:
            resp = input(f"Add runs for {participant} — {display_condition(cond_label)}? [Y/n]: ").strip().lower()
            if resp.startswith("n"):
                continue
            paths = select_runs_for_condition(
                participant,
                cond_label,
                expected=2,
                existing_runs=existing_runs + runs,
            )
            for pth in paths:
                runs.append(dict(participant=participant, condition=cond_label, path=pth))
    return runs

def reselect_runs_for_participants(participants, conditions, existing_runs=None):
    """
    Directly prompt file selection for each condition (no extra yes/no gate).
    Useful when user explicitly asked to reselect/add these participants.
    """
    runs = []
    existing_runs = list(existing_runs) if existing_runs else []
    for participant in participants:
        print(f"\nSelecting runs for {participant}")
        for cond_label in conditions:
            paths = select_runs_for_condition(
                participant,
                cond_label,
                expected=2,
                existing_runs=existing_runs + runs,
            )
            for pth in paths:
                runs.append(dict(participant=participant, condition=cond_label, path=pth))
    return runs

def gather_runs(existing_runs=None):

    runs = []
    existing_runs = list(existing_runs) if existing_runs else []
    print("Enter each participant, then pick up to two runs per condition (FPV only / FPV + minimap / FPV + haptic).")
    print("Leave participant blank to finish.")
    while True:
        participant = input("\nParticipant ID: ").strip()
        if participant == "":
            break

        for cond_label in CONDITIONS:
            resp = input(f"Add runs for {display_condition(cond_label)}? [Y/n]: ").strip().lower()
            if resp.startswith("n"):
                continue
            paths = select_runs_for_condition(
                participant,
                cond_label,
                expected=2,
                existing_runs=existing_runs + runs,
            )
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
        return dedupe_runs(runs)
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

def _cache_item_to_record(item: dict):
    metrics = item.get("metrics", {}) if isinstance(item, dict) else {}
    participant = item.get("participant", "") if isinstance(item, dict) else ""
    condition = item.get("condition", "") if isinstance(item, dict) else ""
    return {
        "Participant": display_pid(participant),
        "Condition": condition,
        "Time_s": metrics.get("run_total_spent_time_s", float("nan")),
        "AvgDist_m": metrics.get("avg_centroid_ref_dist_m", float("nan")),
        "Survived": metrics.get("survivors", float("nan")),
        "Disconnected": metrics.get("disconnected", float("nan")),
        "Crashed": metrics.get("crashed_total", float("nan")),
        "Split": metrics.get("split_metric", float("nan")),
        "Sweep_pct": metrics.get("overall_sweep_pct", float("nan")),
    }

def save_metrics_cache(items):
    serializable = []
    for it in items:
        serializable.append({
            "participant": it.get("participant", ""),
            "condition": it.get("condition", ""),
            "path": str(it.get("path", "")),
            "metrics": it.get("metrics", {}),
        })
    try:
        METRICS_CACHE.write_text(json.dumps(serializable, indent=2))
        print(f"\nSaved metrics cache to {METRICS_CACHE}")
    except Exception as e:
        print(f"\nCould not save metrics cache ({e})")

def load_metrics_cache():
    if not METRICS_CACHE.exists():
        return []
    try:
        data = json.loads(METRICS_CACHE.read_text())
        out = []
        if isinstance(data, list):
            for it in data:
                if not isinstance(it, dict):
                    continue
                p = it.get("participant")
                c = it.get("condition")
                path = it.get("path")
                m = it.get("metrics")
                if not (p and c and path and isinstance(m, dict)):
                    continue
                out.append({"participant": p, "condition": c, "path": Path(path), "metrics": m})
        return out
    except Exception as e:
        print(f"Warning: could not load metrics cache ({e}); ignoring metrics cache.")
        return []

def _cache_items_to_runs(items):
    runs = []
    for it in items:
        runs.append({
            "participant": it.get("participant", ""),
            "condition": it.get("condition", ""),
            "path": Path(it.get("path", "")),
        })
    return dedupe_runs(runs)

def _dedupe_cache_items(items):
    out = []
    seen = set()
    for it in items:
        p = it.get("path")
        if p is None:
            continue
        key = str(normalize_path(Path(p)))
        if key in seen:
            continue
        seen.add(key)
        out.append(it)
    return out

def _compute_cache_items_from_runs(runs):
    items = []
    for run in runs:
        json_path = Path(run["path"])
        if not json_path.exists():
            print(f"  Skipping missing file: {json_path}")
            continue
        metrics = fetch_metrics(json_path)
        if not metrics:
            continue
        items.append({
            "participant": run["participant"],
            "condition": run["condition"],
            "path": json_path,
            "metrics": metrics,
        })
    return items

def _records_from_cache_items(items):
    return [_cache_item_to_record(it) for it in items]

def _participant_from_token(token: str, known_map: dict):
    can = canonical_pid(token)
    if can in known_map:
        return known_map[can]
    return token

def load_selection_cache():
    if not SELECTION_CACHE.exists():
        return set()
    try:
        data = json.loads(SELECTION_CACHE.read_text())
    except Exception:
        return set()
    out = set()
    if not isinstance(data, list):
        return out
    for it in data:
        if not isinstance(it, dict):
            continue
        can = canonical_pid(it.get("participant_can", ""))
        cond = str(it.get("condition", "")).strip()
        if can and cond in CONDITIONS:
            out.add((can, cond))
    return out

def save_selection_cache(selection_set):
    serializable = [
        {"participant_can": can, "condition": cond}
        for (can, cond) in sorted(selection_set, key=lambda x: (x[1], x[0]))
    ]
    try:
        SELECTION_CACHE.write_text(json.dumps(serializable, indent=2))
    except Exception:
        pass

def _participant_sort_key(pid: str):
    can = canonical_pid(pid)
    if can.isdigit():
        return (0, int(can))
    return (1, can.lower())

def select_participants_gui(records):
    """
    Let user choose participant-condition entries in a 3-column GUI.
    Returns a set of (canonical_pid, condition), or None if canceled.
    """
    if tk is None:
        return None

    cond_to_pids = {}
    for cond in CONDITIONS:
        pids = sorted(
            {
                str(r.get("Participant", "")).strip()
                for r in records
                if str(r.get("Condition", "")).strip() == cond and str(r.get("Participant", "")).strip()
            },
            key=_participant_sort_key,
        )
        cond_to_pids[cond] = pids

    state = {"result": None}
    saved_selection = load_selection_cache()
    root = tk.Tk()
    root.title("Select Participants for Plot")
    root.geometry("+80+80")
    root.minsize(860, 480)
    # Keep selector visible above other windows while the user is choosing.
    root.update_idletasks()
    root.lift()
    try:
        root.attributes("-topmost", True)
    except Exception:
        pass
    try:
        root.focus_force()
    except Exception:
        pass

    info = tk.Label(
        root,
        text="Select participant IDs for each category. Only selected entries are used for plotting/statistics.",
        anchor="w",
        justify="left",
    )
    info.grid(row=0, column=0, columnspan=3, sticky="ew", padx=10, pady=(10, 2))

    listboxes = {}
    matched_saved = 0
    for i, cond in enumerate(CONDITIONS):
        frame = tk.LabelFrame(root, text=f"{display_condition(cond)} ({len(cond_to_pids[cond])})", padx=6, pady=6)
        frame.grid(row=1, column=i, sticky="nsew", padx=8, pady=8)
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        lb = tk.Listbox(frame, selectmode=tk.MULTIPLE, exportselection=False, height=18, width=26)
        lb.grid(row=0, column=0, sticky="nsew")
        sb = tk.Scrollbar(frame, orient="vertical", command=lb.yview)
        sb.grid(row=0, column=1, sticky="ns")
        lb.config(yscrollcommand=sb.set)

        pids = cond_to_pids[cond]
        if not pids:
            lb.insert(tk.END, "(none)")
            lb.config(state=tk.DISABLED)
        else:
            for p in pids:
                lb.insert(tk.END, p)
            if saved_selection:
                for idx, p in enumerate(pids):
                    if (canonical_pid(p), cond) in saved_selection:
                        lb.select_set(idx)
                        matched_saved += 1
            else:
                lb.select_set(0, tk.END)
        listboxes[cond] = (lb, pids)

    # If a cache exists but nothing matched this dataset, keep safe default = all selected.
    if saved_selection and matched_saved == 0:
        for lb, pids in listboxes.values():
            if pids:
                lb.select_set(0, tk.END)
        print("Previous participant selection found, but none matched current data; selected all by default.")
    elif saved_selection:
        print(f"Loaded previous participant selection: matched {matched_saved} entries.")

    root.rowconfigure(1, weight=1)
    for c in range(3):
        root.columnconfigure(c, weight=1)

    button_bar = tk.Frame(root)
    button_bar.grid(row=2, column=0, columnspan=3, sticky="ew", padx=10, pady=(0, 10))

    def _close_selector():
        try:
            root.attributes("-topmost", False)
        except Exception:
            pass
        try:
            root.grab_release()
        except Exception:
            pass
        try:
            root.withdraw()
        except Exception:
            pass
        try:
            root.destroy()
        except Exception:
            pass

    def _select_all():
        for lb, pids in listboxes.values():
            if pids:
                lb.select_set(0, tk.END)

    def _clear_all():
        for lb, pids in listboxes.values():
            if pids:
                lb.selection_clear(0, tk.END)

    def _apply():
        chosen = set()
        for cond, (lb, pids) in listboxes.items():
            for idx in lb.curselection():
                if 0 <= idx < len(pids):
                    chosen.add((canonical_pid(pids[idx]), cond))
        if not chosen:
            if messagebox is not None:
                messagebox.showwarning("No selection", "Select at least one participant entry.")
            return
        state["result"] = chosen
        save_selection_cache(chosen)
        _close_selector()

    def _cancel():
        state["result"] = None
        _close_selector()

    tk.Button(button_bar, text="Select all", command=_select_all).pack(side="left", padx=4)
    tk.Button(button_bar, text="Clear all", command=_clear_all).pack(side="left", padx=4)
    tk.Button(button_bar, text="Apply", command=_apply).pack(side="right", padx=4)
    tk.Button(button_bar, text="Cancel", command=_cancel).pack(side="right", padx=4)

    root.protocol("WM_DELETE_WINDOW", _cancel)
    try:
        root.grab_set()
    except Exception:
        pass
    try:
        root.wait_window()
    except Exception:
        pass
    # Safety cleanup in case the callback path did not fully tear down on this platform.
    _close_selector()
    try:
        tk._default_root = None
    except Exception:
        pass
    return state["result"]

def filter_cache_items_with_gui(items):
    records = _records_from_cache_items(items)
    selected = select_participants_gui(records)
    if selected is None:
        print("Participant selection canceled; keeping all participants.")
        return items
    kept = [
        it for it in items
        if (canonical_pid(it.get("participant", "")), str(it.get("condition", ""))) in selected
    ]
    print(f"Participant GUI selection kept {len(kept)} run(s) from {len(items)}.")
    return kept

def _traj_utc_window_seconds(json_path: Path):
    try:
        data = json.loads(Path(json_path).read_text())
    except Exception:
        return None, None
    start_ms = data.get("utcStartMs")
    end_ms = data.get("utcEndMs")
    if isinstance(start_ms, (int, float)) and isinstance(end_ms, (int, float)):
        return float(start_ms) / 1000.0, float(end_ms) / 1000.0

    vals = []
    for tr in data.get("trajectories", []) if isinstance(data, dict) else []:
        for fr in tr.get("frames", []) if isinstance(tr, dict) else []:
            ms = fr.get("utcMs")
            if isinstance(ms, (int, float)):
                vals.append(float(ms) / 1000.0)
    if not vals:
        return None, None
    return float(min(vals)), float(max(vals))

def _read_first_csv_utc_seconds(csv_path: Path):
    try:
        df = pd.read_csv(csv_path, usecols=["utc_time"], nrows=400)
    except Exception:
        return None
    if df.empty or "utc_time" not in df.columns:
        return None
    ts = pd.to_datetime(df["utc_time"], utc=True, errors="coerce")
    ts = ts.dropna()
    if ts.empty:
        return None
    return float(ts.iloc[0].value) / 1e9

def _pick_gaze_csv_for_traj(traj_path: Path, traj_start_s, first_utc_cache: dict):
    folder = Path(traj_path).parent
    candidates = sorted(folder.glob("*.csv"))
    if not candidates:
        return None
    if len(candidates) == 1 or traj_start_s is None:
        return max(candidates, key=lambda p: p.stat().st_mtime)

    best = None
    best_delta = float("inf")
    for p in candidates:
        key = str(normalize_path(p))
        if key not in first_utc_cache:
            first_utc_cache[key] = _read_first_csv_utc_seconds(p)
        t0 = first_utc_cache.get(key)
        if t0 is None:
            continue
        d = abs(float(t0) - float(traj_start_s))
        if d < best_delta:
            best_delta = d
            best = p
    if best is not None:
        return best
    return max(candidates, key=lambda p: p.stat().st_mtime)

def _load_gaze_samples(csv_path: Path):
    required = ["utc_time", "x", "y"]
    try:
        df = pd.read_csv(csv_path)
    except Exception:
        return None
    if any(c not in df.columns for c in required):
        return None
    if "is_lost" in df.columns:
        lost = pd.to_numeric(df["is_lost"], errors="coerce")
        df = df[lost == 0]
    if df.empty:
        return None

    ts = pd.to_datetime(df["utc_time"], utc=True, errors="coerce")
    x = pd.to_numeric(df["x"], errors="coerce")
    y = pd.to_numeric(df["y"], errors="coerce")
    tmp = pd.DataFrame({"utc_time": ts, "x": x, "y": y})
    tmp = tmp.replace([np.inf, -np.inf], np.nan).dropna(subset=["utc_time", "x", "y"])
    if tmp.empty:
        return None
    out = pd.DataFrame({
        "utc_s": tmp["utc_time"].astype("int64") / 1e9,
        "x": tmp["x"],
        "y": tmp["y"],
    })
    if out.empty:
        return None
    return out

def _safe_entropy_from_hist(hist):
    total = float(hist.sum())
    if total <= 0:
        return float("nan"), float("nan")
    p = hist / total
    pnz = p[p > 0]
    ent = float(-(pnz * np.log2(pnz)).sum())
    max_ent = np.log2(hist.size) if hist.size > 1 else 1.0
    ent_norm = ent / max_ent if max_ent > 0 else float("nan")
    return ent, ent_norm

def _holm_correction(pvals):
    m = len(pvals)
    if m == 0:
        return []
    order = sorted(range(m), key=lambda i: pvals[i])
    adjusted = [1.0] * m
    prev = 0.0
    for rank, idx in enumerate(order):
        p_adj = min(1.0, (m - rank) * float(pvals[idx]))
        p_adj = max(p_adj, prev)
        adjusted[idx] = p_adj
        prev = p_adj
    return adjusted

def p_to_stars(p):
    if p < 0.001:
        return "***"
    if p < 0.01:
        return "**"
    if p < 0.05:
        return "*"
    return ""

def _beeswarm_offsets(values, max_width=0.22, n_bins=28):
    """
    Return deterministic x-offsets for a simple beeswarm look.
    Points with similar y are spread left/right to reduce overlap.
    """
    v = np.asarray(values, dtype=float)
    n = len(v)
    if n <= 1:
        return np.zeros(n, dtype=float)
    vmin = float(np.min(v))
    vmax = float(np.max(v))
    span = max(vmax - vmin, 1e-9)
    bin_h = span / float(max(int(n_bins), 1))
    bins = np.floor((v - vmin) / bin_h).astype(int)

    offsets = np.zeros(n, dtype=float)
    pattern = [0]
    for k in range(1, n + 2):
        pattern.extend([k, -k])

    # Deterministic within-bin ordering by value then original index.
    by_bin = {}
    for i, b in enumerate(bins.tolist()):
        by_bin.setdefault(b, []).append(i)
    for _, idxs in by_bin.items():
        idxs.sort(key=lambda i: (v[i], i))
        m = len(idxs)
        if m <= 1:
            offsets[idxs[0]] = 0.0
            continue
        step = min(max_width / max(m // 2, 1), 0.055)
        for j, i in enumerate(idxs):
            lane = pattern[j]
            offsets[i] = lane * step

    return np.clip(offsets, -max_width, max_width)

def _run_sort_time_seconds(json_path: Path):
    """Best-effort run timestamp from filename, then UTC in JSON, then file mtime."""
    try:
        m = re.search(r"(\d{8}_\d{6})", json_path.name)
        if m:
            dt = datetime.strptime(m.group(1), "%Y%m%d_%H%M%S").replace(tzinfo=timezone.utc)
            return float(dt.timestamp())
    except Exception:
        pass

    t0, _ = _traj_utc_window_seconds(json_path)
    if t0 is not None:
        return float(t0)
    try:
        return float(json_path.stat().st_mtime)
    except Exception:
        return float("inf")

def _to_float_or_nan(v):
    try:
        return float(v)
    except Exception:
        return float("nan")

def compute_learning_curve_pairs(items, excluded_participants):
    """
    Build first-vs-second run learning pairs per participant+condition.
    Uses selected runs after GUI filtering.
    """
    rows = []
    for it in items:
        pid_can = canonical_pid(it.get("participant", ""))
        if pid_can in excluded_participants:
            continue
        cond = str(it.get("condition", "")).strip()
        if cond not in CONDITIONS:
            continue
        p = Path(it.get("path", ""))
        metrics = it.get("metrics", {}) if isinstance(it.get("metrics", {}), dict) else {}
        rows.append({
            "participant": display_pid(it.get("participant", "")),
            "participant_can": pid_can,
            "condition": cond,
            "path": str(p),
            "order_s": _run_sort_time_seconds(p),
            "time_s": _to_float_or_nan(metrics.get("run_total_spent_time_s")),
            "avgdist_m": _to_float_or_nan(metrics.get("avg_centroid_ref_dist_m")),
            "sweep_pct": _to_float_or_nan(metrics.get("overall_sweep_pct")),
            "split": _to_float_or_nan(metrics.get("split_metric")),
            "disconnected": _to_float_or_nan(metrics.get("disconnected")),
            "crashed": _to_float_or_nan(metrics.get("crashed_total")),
        })

    if not rows:
        return pd.DataFrame()

    run_df = pd.DataFrame(rows)
    pairs = []
    for (pid, cond), grp in run_df.groupby(["participant", "condition"], sort=False):
        grp = grp.sort_values("order_s")
        if len(grp) < 2:
            continue
        first = grp.iloc[0]
        second = grp.iloc[1]
        pairs.append({
            "Participant": pid,
            "Condition": cond,
            "first_path": first["path"],
            "second_path": second["path"],
            "first_time_s": first["time_s"],
            "second_time_s": second["time_s"],
            "improve_time_s": first["time_s"] - second["time_s"],  # + means faster second run
            "first_avgdist_m": first["avgdist_m"],
            "second_avgdist_m": second["avgdist_m"],
            "improve_avgdist_m": first["avgdist_m"] - second["avgdist_m"],  # + means lower error second run
            "first_sweep_pct": first["sweep_pct"],
            "second_sweep_pct": second["sweep_pct"],
            "improve_sweep_pct": second["sweep_pct"] - first["sweep_pct"],  # + means better coverage second run
            "first_disconnected": first["disconnected"],
            "second_disconnected": second["disconnected"],
            "improve_disconnected": first["disconnected"] - second["disconnected"],
            "first_crashed": first["crashed"],
            "second_crashed": second["crashed"],
            "improve_crashed": first["crashed"] - second["crashed"],
            "first_split": first["split"],
            "second_split": second["split"],
            "delta_split": second["split"] - first["split"],
            "improve_split": first["split"] - second["split"],  # + means lower split imbalance in run 2
        })
    return pd.DataFrame(pairs)

analysis_items = None
cached_items = load_metrics_cache()
if cached_items:
    cached_items = _dedupe_cache_items(cached_items)
    print(f"\nFound saved metrics cache: {METRICS_CACHE} ({len(cached_items)} runs)")
    for i, it in enumerate(cached_items[:8], start=1):
        print(f"  {i}. {it['participant']} — {it['condition']} — {it['path']}")
    if len(cached_items) > 8:
        print(f"  ... and {len(cached_items) - 8} more")
    participant_counts = {}
    for it in cached_items:
        pid = str(it.get("participant", "")).strip()
        if not pid:
            continue
        participant_counts[pid] = participant_counts.get(pid, 0) + 1
    if participant_counts:
        print("Participants already included (full IDs):")
        for pid in sorted(participant_counts.keys(), key=lambda s: canonical_pid(s)):
            print(f"  - {pid} ({participant_counts[pid]} run(s))")

    resp = input("Need to reselect files for certain participant or add new participant? [y/N]: ").strip().lower()
    if not resp.startswith("y"):
        analysis_items = list(cached_items)
    else:
        known = {}
        for it in cached_items:
            can = canonical_pid(it.get("participant", ""))
            if can and can not in known:
                known[can] = it.get("participant", "")

        selected_participants = []
        selected_set = set()
        print("Enter participant IDs one by one to reselect/add.")
        while True:
            tok = input("Participant ID: ").strip()
            if tok == "":
                print("  Empty ID ignored.")
            else:
                picked = _participant_from_token(tok, known)
                can = canonical_pid(picked)
                if can in selected_set:
                    print(f"  Already selected: {picked}")
                else:
                    selected_participants.append(picked)
                    selected_set.add(can)
                    if canonical_pid(tok) not in known:
                        print(f"  New participant to add: {picked}")
                    else:
                        print(f"  Will reselect: {picked}")
            add_next = input("Add next participant ID? [y/N]: ").strip().lower()
            if not add_next.startswith("y"):
                break

        if not selected_participants:
            analysis_items = list(cached_items)
        else:
            kept_cached = [it for it in cached_items if canonical_pid(it.get("participant", "")) not in selected_set]
            seed_runs = _cache_items_to_runs(kept_cached)

            print("\nReselect runs for the listed participants.")
            reselection_runs = reselect_runs_for_participants(
                selected_participants,
                CONDITIONS,
                existing_runs=seed_runs,
            )
            new_items = _compute_cache_items_from_runs(reselection_runs)

            merged = list(kept_cached)
            for p in selected_participants:
                can = canonical_pid(p)
                new_for_p = [it for it in new_items if canonical_pid(it.get("participant", "")) == can]
                if new_for_p:
                    merged.extend(new_for_p)
                else:
                    old_for_p = [it for it in cached_items if canonical_pid(it.get("participant", "")) == can]
                    if old_for_p:
                        print(f"  No new runs selected for {p}; keeping cached entries.")
                        merged.extend(old_for_p)

            merged = _dedupe_cache_items(merged)
            save_metrics_cache(merged)
            analysis_items = merged

if analysis_items is None:
    # Gather runs (reuse cache if desired) and compute metrics on the fly via plot_traj_15.py
    saved = maybe_use_saved_runs()
    runs = []
    if saved is not None:
        runs = dedupe_runs(saved)
        saved_participants = sorted({r["participant"] for r in saved})
        add_minimap = input("Add FPV + minimap runs for existing participants? [Y/n]: ").strip().lower()
        if not add_minimap.startswith("n"):
            runs.extend(add_runs_for_existing_participants(saved_participants, ["Mini Map"], existing_runs=runs))
        add_more = input("Add new participants/runs on top of saved selection? [y/N]: ").strip().lower()
        if add_more.startswith("y"):
            runs.extend(gather_runs(existing_runs=runs))
    else:
        runs = gather_runs()

    runs = dedupe_runs(runs)

    if runs:
        resp = input("Save this selection for next time? [Y/n]: ").strip().lower()
        if not resp.startswith("n"):
            save_runs(runs)

    if not runs:
        print("No runs entered; nothing to plot.")
        sys.exit(0)

    metrics_cache_items = _compute_cache_items_from_runs(runs)
    analysis_items = metrics_cache_items

    if metrics_cache_items:
        resp = input("Save computed performance metrics to JSON cache? [Y/n]: ").strip().lower()
        if not resp.startswith("n"):
            save_metrics_cache(metrics_cache_items)

if not analysis_items:
    print("No valid metrics gathered; aborting plot.")
    sys.exit(1)

analysis_items = filter_cache_items_with_gui(analysis_items)
if not analysis_items:
    print("No records left after GUI selection; aborting plot.")
    sys.exit(1)

records = _records_from_cache_items(analysis_items)

def maybe_exclude_participants(records):
    participants = sorted({str(r.get("Participant", "")).strip() for r in records if str(r.get("Participant", "")).strip()})
    if not participants:
        return records, set()
    print("\nParticipants in current performance data:")
    for p in participants:
        print(f"  - {p}")
    valid_canon = {canonical_pid(p): p for p in participants}

    def _resolve_to_known(pid_text: str):
        can = canonical_pid(pid_text)
        if not can:
            return None
        if can in valid_canon:
            return can
        # Numeric fallback: treat P09 and P9 as the same participant.
        if can.isdigit():
            try:
                num = int(can)
                for v in valid_canon:
                    if v.isdigit() and int(v) == num:
                        return v
            except Exception:
                pass
        return None

    def _tokens(text: str):
        return [t.strip() for t in text.replace(",", " ").split() if t.strip()]

    exclude_set = set()

    def _add_ids(raw_text: str):
        for tok in _tokens(raw_text):
            resolved = _resolve_to_known(tok)
            if resolved is None:
                print(f"  Unknown participant, ignored: {tok}")
                continue
            if resolved in exclude_set:
                print(f"  Already excluded: {display_pid(resolved)}")
                continue
            exclude_set.add(resolved)
            print(f"  Excluded: {display_pid(resolved)}")

    resp = input("Exclude certain participant(s) from analysis? [y/N] (or type IDs like P9,P10): ").strip()
    if resp == "":
        return records, set()
    if resp.lower().startswith("y"):
        while True:
            pid = input("Participant ID(s) to exclude (comma-separated allowed): ").strip()
            if pid:
                _add_ids(pid)
            else:
                print("  Empty ID ignored.")
            more = input("Exclude another participant? [y/N]: ").strip().lower()
            if not more.startswith("y"):
                break
    else:
        _add_ids(resp)
        while True:
            more = input("Exclude another participant? [y/N]: ").strip().lower()
            if not more.startswith("y"):
                break
            pid = input("Participant ID(s) to exclude (comma-separated allowed): ").strip()
            if pid:
                _add_ids(pid)
            else:
                print("  Empty ID ignored.")

    kept = [r for r in records if canonical_pid(r.get("Participant", "")) not in exclude_set]
    removed = len(records) - len(kept)
    print(f"Excluded {removed} record(s).")
    return kept, exclude_set

if ENABLE_POST_GUI_EXCLUSION and sys.stdin.isatty():
    records, excluded_participants = maybe_exclude_participants(records)
    if not records:
        print("No records left after exclusion; aborting plot.")
        sys.exit(1)
else:
    excluded_participants = set()

perf = pd.DataFrame(records)
# Keep an explicit analysis dataframe so significance tests always honor exclusions.
perf_for_stats = perf.copy()
if excluded_participants:
    perf_for_stats = perf_for_stats[
        ~perf_for_stats["Participant"].apply(lambda p: canonical_pid(p) in excluded_participants)
    ].copy()

perf_grp = perf_for_stats.groupby(["Participant", "Condition"]).mean(numeric_only=True).reset_index()

def compute_gaze_distribution_metrics(items):
    """
    Build run-level gaze metrics from selected trajectory items.
    Gaze CSV is auto-matched from the same folder as each trajectory JSON.
    """
    per_run = []
    csv_first_utc_cache = {}
    skipped = 0

    for it in items:
        traj_path = Path(it.get("path", ""))
        if not traj_path.exists():
            skipped += 1
            continue
        start_s, end_s = _traj_utc_window_seconds(traj_path)
        gaze_csv = _pick_gaze_csv_for_traj(traj_path, start_s, csv_first_utc_cache)
        if gaze_csv is None:
            skipped += 1
            continue
        gaze_df = _load_gaze_samples(gaze_csv)
        if gaze_df is None or gaze_df.empty:
            skipped += 1
            continue
        if start_s is not None and end_s is not None:
            gaze_df = gaze_df[(gaze_df["utc_s"] >= start_s) & (gaze_df["utc_s"] <= end_s)]
        if gaze_df.empty:
            skipped += 1
            continue

        x = gaze_df["x"].to_numpy(dtype=float)
        y = gaze_df["y"].to_numpy(dtype=float)
        per_run.append({
            "Participant": display_pid(it.get("participant", "")),
            "Condition": it.get("condition", ""),
            "traj_path": str(traj_path),
            "gaze_csv": str(gaze_csv),
            "n_samples": int(len(gaze_df)),
            "x": x,
            "y": y,
        })

    if not per_run:
        return None, None, None

    all_x = np.concatenate([r["x"] for r in per_run])
    all_y = np.concatenate([r["y"] for r in per_run])
    if len(all_x) == 0 or len(all_y) == 0:
        return None, None, None
    x_min, x_max = float(np.nanmin(all_x)), float(np.nanmax(all_x))
    y_min, y_max = float(np.nanmin(all_y)), float(np.nanmax(all_y))
    if not np.isfinite(x_min) or not np.isfinite(x_max) or not np.isfinite(y_min) or not np.isfinite(y_max):
        return None, None, None
    if x_max <= x_min:
        x_max = x_min + 1.0
    if y_max <= y_min:
        y_max = y_min + 1.0

    cx = 0.5 * (x_min + x_max)
    cy = 0.5 * (y_min + y_max)
    center_w = 0.30 * (x_max - x_min)
    center_h = 0.30 * (y_max - y_min)
    diag = max(float(np.hypot(x_max - x_min, y_max - y_min)), 1e-9)

    for r in per_run:
        hist, _, _ = np.histogram2d(
            r["x"], r["y"],
            bins=[GAZE_BINS, GAZE_BINS],
            range=[[x_min, x_max], [y_min, y_max]],
        )
        ent, ent_norm = _safe_entropy_from_hist(hist)
        in_center = (
            (r["x"] >= cx - 0.5 * center_w) & (r["x"] <= cx + 0.5 * center_w) &
            (r["y"] >= cy - 0.5 * center_h) & (r["y"] <= cy + 0.5 * center_h)
        )
        mean_center_dist = float(np.mean(np.hypot(r["x"] - cx, r["y"] - cy)))
        r["entropy_bits"] = ent
        r["entropy_norm"] = ent_norm
        r["center_dwell_pct"] = 100.0 * float(np.mean(in_center))
        r["center_dist_norm"] = mean_center_dist / diag
        r["hist"] = hist.astype(float)

    rows = []
    for r in per_run:
        rows.append({
            "Participant": r["Participant"],
            "Condition": r["Condition"],
            "n_samples": r["n_samples"],
            "entropy_bits": r["entropy_bits"],
            "entropy_norm": r["entropy_norm"],
            "center_dwell_pct": r["center_dwell_pct"],
            "center_dist_norm": r["center_dist_norm"],
            "traj_path": r["traj_path"],
            "gaze_csv": r["gaze_csv"],
        })
    df_runs = pd.DataFrame(rows)

    cond_hists = {}
    for cond in CONDITIONS:
        h_list = [r["hist"] for r in per_run if r["Condition"] == cond]
        if h_list:
            cond_hists[cond] = np.mean(h_list, axis=0)

    bounds = (x_min, x_max, y_min, y_max)
    if skipped > 0:
        print(f"Gaze analysis: skipped {skipped} run(s) due to missing/invalid gaze data.")
    return df_runs, cond_hists, bounds

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
if excluded_participants:
    subj = subj[~subj["Participant"].apply(lambda p: canonical_pid(p) in excluded_participants)].copy()

participants_obj = sorted(perf_grp["Participant"].unique())
participants_subj = sorted(subj["Participant"].unique())
all_participants = sorted(set(participants_obj) | set(participants_subj))
cmap = plt.get_cmap("tab10")
colors = {p: cmap(i % cmap.N) for i, p in enumerate(all_participants)}

SHOW_COUNTS_ON_FIGURES = prompt_yes_no("Show counts (runs/samples/participants) on figure labels?", default=True)

_label_offsets_pts = [8, -8, 14, -14, 20, -20, 26, -26]
_label_offsets_x = [10, -10, 18, -18, 24, -24, 30, -30]
def label_offset_points(pid: str) -> int:
    """Return a small vertical pixel offset for annotations to reduce overlap."""
    try:
        idx = all_participants.index(pid)
    except ValueError:
        idx = 0
    return _label_offsets_pts[idx % len(_label_offsets_pts)]

def label_offset_x(pid: str) -> int:
    """Return a small horizontal pixel offset for annotations to reduce overlap."""
    try:
        idx = all_participants.index(pid)
    except ValueError:
        idx = 0
    return _label_offsets_x[idx % len(_label_offsets_x)]

def _series_values(series, x_labels):
    if isinstance(series, dict):
        return [series.get(lbl, float("nan")) for lbl in x_labels]
    return list(series)

def annotate_identical_series(ax, series_pairs, colors, x_labels, round_decimals: int = 3):
    """
    Annotate only participants whose entire series across x_labels is identical.
    series_pairs: dict pid -> dict(label -> value) or list/tuple aligned to x_labels
    """
    buckets = {}
    for pid, series in series_pairs.items():
        values = _series_values(series, x_labels)
        if any(pd.isna(v) for v in values):
            continue
        key = tuple(round(float(v), round_decimals) for v in values)
        buckets.setdefault(key, []).append(pid)
    for pids in buckets.values():
        if len(pids) < 2:
            continue
        for i, pid in enumerate(pids):
            dy = 0  # keep same vertical position as the marker
            dx = _label_offsets_x[i % len(_label_offsets_x)]
            values = _series_values(series_pairs[pid], x_labels)
            for label, val in zip(x_labels, values):
                ax.annotate(pid, (label, val),
                            xytext=(dx, dy), textcoords="offset points",
                            ha="center", va="bottom", fontsize=8,
                            color=colors[pid], alpha=0.9)

def annotate_all_points(ax, series_pairs, colors, x_labels):
    """Annotate every point close to its dot; stack labels only when dots overlap."""
    buckets = {}  # key: (x_label, rounded_y) -> [pid, ...]
    for pid, series in series_pairs.items():
        values = _series_values(series, x_labels)
        for label, val in zip(x_labels, values):
            if pd.isna(val):
                continue
            key = (label, round(float(val), 4))
            buckets.setdefault(key, []).append(pid)

    # Symmetric vertical offsets around the point for overlapping labels.
    dy_pattern = [0, 8, -8, 14, -14, 20, -20, 26, -26]
    for (label, y_rounded), pids in buckets.items():
        y = float(y_rounded)
        for i, pid in enumerate(sorted(pids)):
            dy = dy_pattern[i] if i < len(dy_pattern) else (30 + 6 * (i - len(dy_pattern) + 1))
            dx = 4 if (i % 2 == 0) else -4
            ha = "left" if dx > 0 else "right"
            ax.annotate(
                pid,
                (label, y),
                xytext=(dx, dy),
                textcoords="offset points",
                ha=ha,
                va="center",
                fontsize=7,
                color=colors.get(pid, "black"),
                alpha=0.95,
            )

def draw_sig_bracket(ax, x1, x2, y, h, text):
    ax.plot([x1, x1, x2, x2], [y, y + h, y + h, y], lw=1.2, c="k")
    ax.text((x1 + x2) / 2.0, y + h, text, ha="center", va="bottom", fontsize=10, color="k")

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
fig.suptitle("Objective Performance: FPV only vs FPV + minimap vs FPV + haptic", fontsize=14, fontweight="bold")

for ax, metric, title in zip(axes.flat, obj_metrics, obj_titles):
    series_pairs = {}
    seen_series = {}
    cond_counts_for_metric = {
        c: int(perf_grp[(perf_grp["Condition"] == c)][metric].notna().sum())
        for c in CONDITIONS
    }
    cond_labels_disp = [condition_label(c, cond_counts_for_metric.get(c), "n") for c in CONDITIONS]
    for p in participants_obj:
        sub = perf_grp[perf_grp["Participant"] == p].copy()
        sub["Condition"] = pd.Categorical(
            sub["Condition"],
            categories=CONDITIONS,
            ordered=True
        )
        sub = sub.sort_values("Condition")
        vals = sub.set_index("Condition")[metric].to_dict()
        y_vals = [vals.get(label, float("nan")) for label in CONDITIONS]
        pair = tuple(y_vals)
        linestyle = "--" if pair in seen_series else "-"
        seen_series[pair] = seen_series.get(pair, 0) + 1
        ax.plot(cond_labels_disp, y_vals,
                marker="o", linewidth=2, color=colors[p], label=p, linestyle=linestyle)
        series_pairs[p] = dict(zip(cond_labels_disp, y_vals))

    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.set_facecolor("#f9f9f9")
    ax.set_ylabel(title)
    annotate_all_points(ax, series_pairs, colors, cond_labels_disp)

for extra_ax in list(axes.flat)[len(obj_metrics):]:
    extra_ax.axis("off")

first_ax = axes.flat[0]
handles, labels = first_ax.get_legend_handles_labels()
fig.legend(handles, labels, loc="lower center", ncol=min(5, len(all_participants)), frameon=False)
plt.tight_layout(rect=[0, 0.08, 1, 0.94])
objective_path = OUT_DIR / "plot_all6_objective.png"
fig.savefig(objective_path, dpi=150)

# ===== FIGURE 1b: Objective boxplots =====
fig_box, axes_box = plt.subplots(nrows, ncols, figsize=(13, 8))
fig_box.suptitle("Objective Performance Boxplots: FPV only vs FPV + minimap vs FPV + haptic", fontsize=14, fontweight="bold")
mw_results = []
kw_results = []
for ax, metric, title in zip(axes_box.flat, obj_metrics, obj_titles):
    sub_df = perf_for_stats.pivot_table(index="Participant", columns="Condition", values=metric)
    cond_cols = [label for label in CONDITIONS if label in sub_df.columns]
    if cond_cols:
        sub_df = sub_df[cond_cols]
    sub_df.boxplot(ax=ax, widths=0.5, patch_artist=True,
                   boxprops=dict(facecolor="#dddddd", color="#444444"),
                   medianprops=dict(color="black"))
    cond_counts = {c: int(pd.to_numeric(sub_df[c], errors="coerce").dropna().shape[0]) for c in cond_cols}
    ax.set_xticklabels([condition_label(c, cond_counts.get(c), "n") for c in cond_cols], rotation=10)
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.set_facecolor("#fafafa")

    # Kruskal-Wallis across available groups (between-subjects).
    if kruskal is not None and len(cond_cols) >= 2:
        kw_groups = []
        kw_names = []
        for c in cond_cols:
            vals = pd.to_numeric(sub_df[c], errors="coerce").dropna().tolist()
            if len(vals) >= 2:
                kw_groups.append(vals)
                kw_names.append(c)
        if len(kw_groups) >= 2:
            try:
                kw_res = kruskal(*kw_groups)
                kw_results.append({
                    "metric": metric,
                    "groups": kw_names,
                    "h_stat": float(kw_res.statistic),
                    "p_value": float(kw_res.pvalue),
                    "stars": p_to_stars(float(kw_res.pvalue)) or "ns",
                })
            except Exception:
                pass

    # Mann-Whitney U tests across available condition pairs.
    if mannwhitneyu is not None and len(cond_cols) >= 2:
        y_candidates = []
        for c in cond_cols:
            vals = pd.to_numeric(sub_df[c], errors="coerce").dropna().tolist()
            y_candidates.extend(vals)
        if y_candidates:
            y_min = min(y_candidates)
            y_max = max(y_candidates)
            y_span = max(y_max - y_min, 1e-9)
            base_y = y_max + 0.06 * y_span
            step = 0.08 * y_span
            h = 0.03 * y_span
            star_level = 0

            for c1, c2 in combinations(cond_cols, 2):
                v1 = pd.to_numeric(sub_df[c1], errors="coerce").dropna()
                v2 = pd.to_numeric(sub_df[c2], errors="coerce").dropna()
                if len(v1) == 0 or len(v2) == 0:
                    continue
                try:
                    res_mw = mannwhitneyu(v1, v2, alternative="two-sided")
                    u_stat = float(res_mw.statistic)
                    p_val = float(res_mw.pvalue)
                except Exception:
                    continue
                stars = p_to_stars(p_val)
                mw_results.append({
                    "metric": metric,
                    "title": title,
                    "group_a": c1,
                    "group_b": c2,
                    "n_a": int(len(v1)),
                    "n_b": int(len(v2)),
                    "u_stat": u_stat,
                    "p_value": p_val,
                    "stars": stars if stars else "ns",
                })
                if stars:
                    x1 = cond_cols.index(c1) + 1
                    x2 = cond_cols.index(c2) + 1
                    y = base_y + star_level * step
                    draw_sig_bracket(ax, x1, x2, y, h, stars)
                    star_level += 1
            if star_level > 0:
                ax.set_ylim(top=base_y + star_level * step + 2 * h)

for extra_ax in list(axes_box.flat)[len(obj_metrics):]:
    extra_ax.axis("off")

plt.tight_layout(rect=[0, 0.05, 1, 0.95])
objective_box_path = OUT_DIR / "plot_all6_objective_boxplot.png"
fig_box.savefig(objective_box_path, dpi=150)

# ===== FIGURE 1c: Objective violin plots =====
fig_vln, axes_vln = plt.subplots(nrows, ncols, figsize=(13, 8))
fig_vln.suptitle("Objective Performance Violin Plots: FPV only vs FPV + minimap vs FPV + haptic", fontsize=14, fontweight="bold")
fig_vln.text(0.5, 0.955, "Red dashed = mean, Blue solid = median", ha="center", va="center", fontsize=9)
for ax, metric, title in zip(axes_vln.flat, obj_metrics, obj_titles):
    sub_df = perf_for_stats.pivot_table(index="Participant", columns="Condition", values=metric)
    cond_cols = [label for label in CONDITIONS if label in sub_df.columns]
    if not cond_cols:
        ax.set_title(f"{title}\n(no data)")
        ax.axis("off")
        continue

    vals_by_cond = [pd.to_numeric(sub_df[c], errors="coerce").dropna().to_numpy(dtype=float) for c in cond_cols]
    pos = np.arange(1, len(cond_cols) + 1)
    # Draw only non-empty groups.
    draw_vals = []
    draw_pos = []
    draw_labels = []
    for p, c, v in zip(pos, cond_cols, vals_by_cond):
        if len(v) == 0:
            continue
        draw_pos.append(p)
        draw_labels.append(c)
        draw_vals.append(v)
    if not draw_vals:
        ax.set_title(f"{title}\n(no data)")
        ax.axis("off")
        continue

    parts = ax.violinplot(draw_vals, positions=draw_pos, widths=0.7, showmeans=True, showmedians=True, showextrema=True)
    for body in parts["bodies"]:
        body.set_facecolor("#9ecae1")
        body.set_edgecolor("#3182bd")
        body.set_alpha(0.55)
    if "cmeans" in parts:
        parts["cmeans"].set_color("#d62728")
        parts["cmeans"].set_linestyle("--")
        parts["cmeans"].set_linewidth(1.4)
    if "cmedians" in parts:
        parts["cmedians"].set_color("#08519c")
        parts["cmedians"].set_linestyle("-")
        parts["cmedians"].set_linewidth(1.7)

    for p, vals in zip(draw_pos, draw_vals):
        offsets = _beeswarm_offsets(vals, max_width=0.18, n_bins=24)
        ax.scatter(np.full(len(vals), p, dtype=float) + offsets, vals, s=16, alpha=0.6, color="#1f77b4", zorder=3)

    # Add significance brackets on violin plots (same pairwise test style as boxplot).
    if mannwhitneyu is not None and len(draw_labels) >= 2:
        y_all = np.concatenate(draw_vals)
        if len(y_all) > 0:
            y_min = float(np.min(y_all))
            y_max = float(np.max(y_all))
            y_span = max(y_max - y_min, 1e-9)
            base_y = y_max + 0.06 * y_span
            step = 0.08 * y_span
            h = 0.03 * y_span
            star_level = 0
            for c1, c2 in combinations(draw_labels, 2):
                v1 = pd.to_numeric(sub_df[c1], errors="coerce").dropna()
                v2 = pd.to_numeric(sub_df[c2], errors="coerce").dropna()
                if len(v1) == 0 or len(v2) == 0:
                    continue
                try:
                    res_mw = mannwhitneyu(v1, v2, alternative="two-sided")
                    p_val = float(res_mw.pvalue)
                except Exception:
                    continue
                stars = p_to_stars(p_val)
                if stars:
                    x1 = draw_labels.index(c1) + 1
                    x2 = draw_labels.index(c2) + 1
                    y = base_y + star_level * step
                    draw_sig_bracket(ax, x1, x2, y, h, stars)
                    star_level += 1
            if star_level > 0:
                ax.set_ylim(top=base_y + star_level * step + 2 * h)

    draw_counts = {c: int(len(v)) for c, v in zip(draw_labels, draw_vals)}
    ax.set_xticks(draw_pos)
    ax.set_xticklabels([condition_label(c, draw_counts.get(c), "n") for c in draw_labels], rotation=10)
    ax.set_title(title)
    ax.grid(alpha=0.3)
    ax.set_facecolor("#fafafa")

for extra_ax in list(axes_vln.flat)[len(obj_metrics):]:
    extra_ax.axis("off")

plt.tight_layout(rect=[0, 0.05, 1, 0.95])
objective_violin_path = OUT_DIR / "plot_all6_objective_violin.png"
fig_vln.savefig(objective_violin_path, dpi=150)

if mannwhitneyu is None:
    print("\nMann-Whitney U analysis skipped: scipy is not available.")
else:
    print("\nMann-Whitney U results (two-sided):")
    if excluded_participants:
        excluded_txt = ", ".join(display_pid(p) for p in sorted(excluded_participants))
        print(f"  Excluding participants: {excluded_txt}")
    if not mw_results:
        print("  No valid pairwise comparisons were computed.")
    else:
        for r in mw_results:
            print(
                f"  {r['metric']} ({r['group_a']} vs {r['group_b']}): "
                f"U={r['u_stat']:.3f}, p={r['p_value']:.6g}, {r['stars']} "
                f"[n={r['n_a']} vs {r['n_b']}]"
            )

if kruskal is None:
    print("\nKruskal-Wallis analysis skipped: scipy is not available.")
else:
    print("\nKruskal-Wallis results (objective metrics):")
    if not kw_results:
        print("  No valid group-wise comparisons were computed.")
    else:
        for r in kw_results:
            gtxt = ", ".join(display_condition(g) for g in r["groups"])
            print(
                f"  {r['metric']} ({gtxt}): "
                f"H={r['h_stat']:.3f}, p={r['p_value']:.6g}, {r['stars']}"
            )

learning_curve_path = None
learning_pairs_df = compute_learning_curve_pairs(analysis_items, excluded_participants)
if learning_pairs_df.empty:
    print("\nLearning curve analysis skipped: no participant-condition has both first and second runs.")
else:
    print("\nLearning curve (second run vs first run):")
    print(f"  Paired participant-condition entries: {len(learning_pairs_df)}")
    learn_metrics = [
        ("improve_time_s", "Time improvement (s, + better)"),
        ("improve_avgdist_m", "Distance improvement (m, + better)"),
        ("improve_sweep_pct", "Sweep improvement (% points, + better)"),
        ("improve_disconnected", "Disconnected improvement (+ fewer in run 2)"),
        ("improve_crashed", "Crashed improvement (+ fewer in run 2)"),
        ("improve_split", "Split improvement (+ lower split imbalance in run 2)"),
    ]
    learning_between_rows = []
    for cond in CONDITIONS:
        sub = learning_pairs_df[learning_pairs_df["Condition"] == cond]
        if sub.empty:
            continue
        print(f"  {display_condition(cond)}: n={len(sub)}")
        for col, label in learn_metrics:
            vals = pd.to_numeric(sub[col], errors="coerce").dropna().to_numpy(dtype=float)
            if len(vals) == 0:
                continue
            msg = f"    {label}: mean={np.mean(vals):.3f}, median={np.median(vals):.3f}"
            if wilcoxon is not None and len(vals) >= 2 and np.any(vals != 0):
                try:
                    wres = wilcoxon(vals, alternative="two-sided", zero_method="wilcox")
                    p = float(wres.pvalue)
                    msg += f", Wilcoxon p={p:.6g} ({p_to_stars(p) or 'ns'})"
                except Exception:
                    pass
            print(msg)

    ncols_lc = 3
    nrows_lc = (len(learn_metrics) + ncols_lc - 1) // ncols_lc
    fig_lc, axes_lc = plt.subplots(nrows_lc, ncols_lc, figsize=(16, 8))
    for ax, (col, title) in zip(axes_lc.flat, learn_metrics):
        vals_by_cond = []
        labels = []
        vals_map = {}
        for cond in CONDITIONS:
            vals = pd.to_numeric(
                learning_pairs_df.loc[learning_pairs_df["Condition"] == cond, col],
                errors="coerce",
            ).dropna().to_numpy(dtype=float)
            if len(vals) == 0:
                continue
            vals_by_cond.append(vals)
            labels.append(cond)
            vals_map[cond] = vals
        if not vals_by_cond:
            ax.set_title(f"{title}\n(no paired data)")
            ax.axis("off")
            continue
        bp = ax.boxplot(
            vals_by_cond,
            labels=labels,
            widths=0.5,
            patch_artist=True,
            boxprops=dict(facecolor="#d9d9d9", color="#444444"),
            medianprops=dict(color="black"),
        )
        # Keep raw dots visible above box elements.
        for b in bp.get("boxes", []):
            b.set_alpha(0.45)
            b.set_zorder(1)
        for w in bp.get("whiskers", []):
            w.set_zorder(1.2)
        for c in bp.get("caps", []):
            c.set_zorder(1.2)
        for m in bp.get("medians", []):
            m.set_zorder(1.5)
        for i, vals in enumerate(vals_by_cond, start=1):
            offsets = _beeswarm_offsets(vals, max_width=0.20, n_bins=26)
            ax.scatter(
                np.full(len(vals), i, dtype=float) + offsets,
                vals,
                s=26,
                alpha=0.9,
                zorder=3,
                edgecolors="white",
                linewidths=0.4,
                color="#1f77b4",
            )
        label_counts = {c: int(len(v)) for c, v in zip(labels, vals_by_cond)}
        ax.set_xticklabels([condition_label(c, label_counts.get(c), "n") for c in labels], rotation=10)

        # Between-condition tests on learning gains.
        groups = [vals_map[c] for c in labels if len(vals_map[c]) >= 2]
        group_names = [c for c in labels if len(vals_map[c]) >= 2]
        kw_h = None
        kw_p = None
        if kruskal is not None and len(groups) >= 2:
            try:
                kw_res = kruskal(*groups)
                kw_h = float(kw_res.statistic)
                kw_p = float(kw_res.pvalue)
            except Exception:
                kw_h, kw_p = None, None
        learning_between_rows.append({
            "metric": col,
            "title": title,
            "test": "Kruskal-Wallis",
            "group_a": "all",
            "group_b": "",
            "stat": kw_h,
            "p_raw": kw_p,
            "p_adj": kw_p,
            "stars": p_to_stars(kw_p) if kw_p is not None else "na",
            "n_a": None,
            "n_b": None,
        })

        pair_rows = []
        if mannwhitneyu is not None and len(group_names) >= 2:
            raw_pvals = []
            raw_rows = []
            for c1, c2 in combinations(group_names, 2):
                v1 = vals_map[c1]
                v2 = vals_map[c2]
                if len(v1) < 2 or len(v2) < 2:
                    continue
                try:
                    mw_res = mannwhitneyu(v1, v2, alternative="two-sided")
                except Exception:
                    continue
                raw_pvals.append(float(mw_res.pvalue))
                raw_rows.append({
                    "metric": col,
                    "title": title,
                    "test": "Mann-Whitney U",
                    "group_a": c1,
                    "group_b": c2,
                    "stat": float(mw_res.statistic),
                    "p_raw": float(mw_res.pvalue),
                    "n_a": int(len(v1)),
                    "n_b": int(len(v2)),
                })
            adj_pvals = _holm_correction(raw_pvals)
            for row, p_adj in zip(raw_rows, adj_pvals):
                row["p_adj"] = float(p_adj)
                row["stars"] = p_to_stars(float(p_adj)) or "ns"
                learning_between_rows.append(row)
                pair_rows.append(row)

        sig_pairs = [r for r in pair_rows if r.get("p_adj") is not None and float(r["p_adj"]) < 0.05]
        if sig_pairs:
            y_all = np.concatenate(vals_by_cond)
            y_min = float(np.min(y_all))
            y_max = float(np.max(y_all))
            y_span = max(y_max - y_min, 1e-9)
            y_base = y_max + 0.06 * y_span
            y_step = 0.08 * y_span
            h = 0.03 * y_span
            for idx_sig, row in enumerate(sig_pairs):
                x1 = labels.index(row["group_a"]) + 1
                x2 = labels.index(row["group_b"]) + 1
                y = y_base + idx_sig * y_step
                draw_sig_bracket(ax, x1, x2, y, h, p_to_stars(row["p_adj"]))
            ax.set_ylim(top=y_base + len(sig_pairs) * y_step + 2 * h)

        ax.axhline(0.0, color="k", linewidth=1, alpha=0.5)
        ax.set_title(title)
        ax.grid(alpha=0.3)
        ax.set_facecolor("#fafafa")
    for extra_ax in list(axes_lc.flat)[len(learn_metrics):]:
        extra_ax.axis("off")
    fig_lc.suptitle("Learning Curve: Run 2 minus Run 1 (or improvement where noted)", fontsize=13, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    learning_curve_path = OUT_LEARNING_CURVE_BOXPLOT
    fig_lc.savefig(learning_curve_path, dpi=150)

    if kruskal is None or mannwhitneyu is None:
        print("  Learning between-condition significance skipped: scipy is not available.")
    else:
        print("  Learning between-condition significance (Kruskal + Holm-corrected pairwise MW):")
        for row in learning_between_rows:
            if row["test"] == "Kruskal-Wallis":
                if row["stat"] is None or row["p_raw"] is None:
                    print(f"    {row['metric']}: Kruskal-Wallis not available")
                else:
                    print(
                        f"    {row['metric']}: H={row['stat']:.3f}, p={row['p_raw']:.6g}, "
                        f"{row['stars'] or 'ns'}"
                    )
            else:
                print(
                    f"    {row['metric']} ({row['group_a']} vs {row['group_b']}): "
                    f"U={row['stat']:.3f}, p_raw={row['p_raw']:.6g}, p_holm={row['p_adj']:.6g}, "
                    f"{row['stars']} [n={row['n_a']} vs {row['n_b']}]"
                )

gaze_heatmap_path = None
gaze_metrics_box_path = None
gaze_stats_rows = []
plot_gaze_analysis = prompt_yes_no("Plot gaze distribution data/metrics?", default=True)
if plot_gaze_analysis:
    analysis_items_for_gaze = [
        it for it in analysis_items
        if canonical_pid(it.get("participant", "")) not in excluded_participants
    ]
    gaze_runs_df, cond_hists, gaze_bounds = compute_gaze_distribution_metrics(analysis_items_for_gaze)
else:
    gaze_runs_df, cond_hists, gaze_bounds = None, None, None

if not plot_gaze_analysis:
    print("\nGaze distribution analysis skipped by user.")
elif gaze_runs_df is None or gaze_runs_df.empty or not cond_hists:
    print("\nGaze distribution analysis skipped: no valid matched gaze data for selected runs.")
else:
    # Average duplicated runs per participant-condition to avoid overweighting individuals with more runs.
    gaze_part_df = gaze_runs_df.groupby(["Participant", "Condition"], as_index=False).agg({
        "n_samples": "mean",
        "entropy_bits": "mean",
        "entropy_norm": "mean",
        "center_dwell_pct": "mean",
        "center_dist_norm": "mean",
    })

    # Group-level heatmaps with a shared color scale.
    x_min, x_max, y_min, y_max = gaze_bounds
    xedges = np.linspace(x_min, x_max, GAZE_BINS + 1)
    x_centers = 0.5 * (xedges[:-1] + xedges[1:])
    x_widths = np.diff(xedges) * 0.9
    y_span = max(y_max - y_min, 1e-9)
    fig_gaze_hm, axes_gaze_hm = plt.subplots(1, 3, figsize=(14, 4))
    vmax = max(float(np.max(h)) for h in cond_hists.values()) if cond_hists else 1.0
    all_density_vals = np.concatenate([h.ravel() for h in cond_hists.values()]) if cond_hists else np.array([0.0])
    positive_vals = all_density_vals[all_density_vals > 0]
    # Robust vmax avoids one hot bin making the full map look too dark.
    if positive_vals.size > 0:
        vmax_robust = float(np.percentile(positive_vals, 99.5))
        vmax_plot = max(min(vmax, vmax_robust), 1e-9)
    else:
        vmax_plot = max(vmax, 1e-9)
    density_norm = PowerNorm(gamma=GAZE_DENSITY_GAMMA, vmin=0.0, vmax=vmax_plot)
    image_for_cb = None
    for i, cond in enumerate(CONDITIONS):
        ax = axes_gaze_hm[i]
        h = cond_hists.get(cond)
        if h is None:
            ax.set_title(f"{display_condition(cond)}\n(no data)")
            ax.axis("off")
            continue
        image_for_cb = ax.imshow(
            h.T,
            origin="lower",
            extent=[x_min, x_max, y_min, y_max],
            cmap="viridis",
            interpolation="bicubic",
            norm=density_norm,
            aspect="equal",
        )
        # Horizontal distribution bars (x-binned) like plot_traj_15:
        # use per-condition x-density profile from the same heatmap bins.
        x_profile = np.sum(h, axis=1)
        x_max_profile = float(np.max(x_profile)) if x_profile.size > 0 else 0.0
        x_norm = (x_profile / x_max_profile) if x_max_profile > 0 else np.zeros_like(x_profile)
        bar_span = 0.40 * y_span
        bar_heights = x_norm * bar_span
        ax.bar(
            x_centers,
            bar_heights,
            width=x_widths,
            bottom=y_min,
            color="#fee8a6",
            edgecolor="none",
            alpha=0.42,
            align="center",
            zorder=3,
        )
        ax.invert_yaxis()
        ax.set_aspect("equal", adjustable="box")
        n_cond = int((gaze_runs_df["Condition"] == cond).sum())
        if SHOW_COUNTS_ON_FIGURES:
            ax.set_title(f"{display_condition(cond)} (n={n_cond} runs)")
        else:
            ax.set_title(display_condition(cond))
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.text(
            0.02, 0.97,
            "Bars: relative horizontal gaze time",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=8,
            color="white",
            bbox=dict(boxstyle="round,pad=0.2", fc=(0, 0, 0, 0.35), ec="none"),
        )
    if image_for_cb is not None:
        fig_gaze_hm.subplots_adjust(left=0.06, right=0.87, bottom=0.12, top=0.86, wspace=0.24)
        cax = fig_gaze_hm.add_axes([0.89, 0.17, 0.015, 0.64])
        cbar = fig_gaze_hm.colorbar(image_for_cb, cax=cax)
        cbar.set_label(f"Average gaze density (power scale, gamma={GAZE_DENSITY_GAMMA:.2f}, vmax@99.5%)")
    fig_gaze_hm.suptitle("Gaze Distribution Heatmaps + Horizontal Bars (UTC-window trimmed, same scale)", fontsize=12, fontweight="bold")
    gaze_heatmap_path = OUT_GAZE_GROUP_HEATMAP
    fig_gaze_hm.savefig(gaze_heatmap_path, dpi=150)

    # Between-subjects stats on gaze metrics: Kruskal-Wallis + Holm-corrected pairwise Mann-Whitney.
    gaze_metric_cols = ["entropy_norm", "center_dist_norm"]
    gaze_metric_titles = {
        "entropy_norm": "Entropy (normalized)",
        "center_dist_norm": "Center Distance (normalized)",
    }
    gaze_ncols = 2
    gaze_nrows = (len(gaze_metric_cols) + gaze_ncols - 1) // gaze_ncols
    fig_gaze_box, axes_gaze_box = plt.subplots(gaze_nrows, gaze_ncols, figsize=(12, 4.8 * gaze_nrows))
    for ax, metric in zip(np.atleast_1d(axes_gaze_box).flat, gaze_metric_cols):
        sub_df = gaze_part_df.pivot_table(index="Participant", columns="Condition", values=metric)
        cond_cols = [c for c in CONDITIONS if c in sub_df.columns]
        if cond_cols:
            sub_df = sub_df[cond_cols]
        vals_by_cond = [pd.to_numeric(sub_df[c], errors="coerce").dropna().to_numpy(dtype=float) for c in cond_cols]
        label_with_n = [
            condition_label(c, len(v), "n") if SHOW_COUNTS_ON_FIGURES else display_condition(c)
            for c, v in zip(cond_cols, vals_by_cond)
        ]
        if not any(len(v) > 0 for v in vals_by_cond):
            ax.set_title(f"{gaze_metric_titles[metric]}\n(no data)")
            ax.axis("off")
            continue
        try:
            ax.boxplot(vals_by_cond, tick_labels=label_with_n, showmeans=True)
        except TypeError:
            ax.boxplot(vals_by_cond, labels=label_with_n, showmeans=True)
        ax.set_title(gaze_metric_titles[metric])
        ax.grid(alpha=0.3)
        ax.tick_params(axis="x", labelrotation=20)
        for lbl in ax.get_xticklabels():
            lbl.set_ha("right")
        ax.set_facecolor("white")

        groups = []
        group_names = []
        for c in cond_cols:
            vals = pd.to_numeric(sub_df[c], errors="coerce").dropna().tolist()
            if len(vals) >= 2:
                groups.append(vals)
                group_names.append(c)

        kw_p = None
        kw_h = None
        if kruskal is not None and len(groups) >= 2:
            try:
                kw_res = kruskal(*groups)
                kw_h = float(kw_res.statistic)
                kw_p = float(kw_res.pvalue)
            except Exception:
                kw_h, kw_p = None, None
        gaze_stats_rows.append({
            "metric": metric,
            "test": "Kruskal-Wallis",
            "group_a": "all",
            "group_b": "",
            "stat": kw_h,
            "p_raw": kw_p,
            "p_adj": kw_p,
            "stars": p_to_stars(kw_p) if kw_p is not None else "na",
        })

        # Pairwise post-hoc only when Kruskal indicates potential difference and MW is available.
        pair_results = []
        if mannwhitneyu is not None and len(cond_cols) >= 2:
            raw_pvals = []
            raw_rows = []
            for c1, c2 in combinations(cond_cols, 2):
                v1 = pd.to_numeric(sub_df[c1], errors="coerce").dropna().tolist()
                v2 = pd.to_numeric(sub_df[c2], errors="coerce").dropna().tolist()
                if len(v1) < 2 or len(v2) < 2:
                    continue
                try:
                    mw_res = mannwhitneyu(v1, v2, alternative="two-sided")
                except Exception:
                    continue
                raw_pvals.append(float(mw_res.pvalue))
                raw_rows.append({
                    "metric": metric,
                    "test": "Mann-Whitney U",
                    "group_a": c1,
                    "group_b": c2,
                    "stat": float(mw_res.statistic),
                    "p_raw": float(mw_res.pvalue),
                    "n_a": len(v1),
                    "n_b": len(v2),
                })

            adj_pvals = _holm_correction(raw_pvals)
            for row, p_adj in zip(raw_rows, adj_pvals):
                row["p_adj"] = float(p_adj)
                row["stars"] = p_to_stars(float(p_adj)) or "ns"
                gaze_stats_rows.append(row)
                pair_results.append(row)

        # Draw significance marks from Holm-adjusted p-values.
        sig_pairs = [r for r in pair_results if r.get("p_adj") is not None and r["p_adj"] < 0.05]
        if sig_pairs:
            y_all = []
            for c in cond_cols:
                vals = pd.to_numeric(sub_df[c], errors="coerce").dropna().tolist()
                y_all.extend(vals)
            if y_all:
                y_min = min(y_all)
                y_max = max(y_all)
                y_span = max(y_max - y_min, 1e-9)
                y_base = y_max + 0.06 * y_span
                y_step = 0.08 * y_span
                h = 0.03 * y_span
                for idx_sig, row in enumerate(sig_pairs):
                    x1 = cond_cols.index(row["group_a"]) + 1
                    x2 = cond_cols.index(row["group_b"]) + 1
                    y = y_base + idx_sig * y_step
                    draw_sig_bracket(ax, x1, x2, y, h, p_to_stars(row["p_adj"]))
                ax.set_ylim(top=y_base + len(sig_pairs) * y_step + 2 * h)

    for extra_ax in list(np.atleast_1d(axes_gaze_box).flat)[len(gaze_metric_cols):]:
        extra_ax.axis("off")

    fig_gaze_box.suptitle("Gaze Distribution Metrics (between-subjects)", fontsize=13, fontweight="bold")
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    gaze_metrics_box_path = OUT_GAZE_METRICS_BOXPLOT
    fig_gaze_box.savefig(gaze_metrics_box_path, dpi=150)

    print("\nGaze distribution stats (between-subjects):")
    for row in gaze_stats_rows:
        if row["test"] == "Kruskal-Wallis":
            if row["stat"] is None or row["p_raw"] is None:
                print(f"  {row['metric']}: Kruskal-Wallis not available")
            else:
                print(
                    f"  {row['metric']}: Kruskal H={row['stat']:.3f}, p={row['p_raw']:.6g}, "
                    f"{row['stars'] or 'ns'}"
                )
        else:
            print(
                f"  {row['metric']} ({row['group_a']} vs {row['group_b']}): "
                f"U={row['stat']:.3f}, p_raw={row['p_raw']:.6g}, p_holm={row['p_adj']:.6g}, "
                f"{row['stars']} [n={row['n_a']} vs {row['n_b']}]"
            )

questionnaire_path = None
questionnaire_box_path = None
if SHOW_QUESTIONNAIRE_PLOTS:
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
        annotate_identical_series(ax, series_pairs, colors, ["Without Haptic", "With Haptic"], round_decimals=2)

    handles_q, labels_q = axes_q[0, 0].get_legend_handles_labels()
    fig_q.legend(handles_q, labels_q, loc="lower center", ncol=max(len(participants_subj), 1), frameon=False)
    plt.tight_layout(rect=[0, 0.12, 1, 0.94])
    questionnaire_path = OUT_DIR / "plot_all6_questionnaire.png"
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
    questionnaire_box_path = OUT_DIR / "plot_all6_questionnaire_boxplot.png"
    fig_qb.savefig(questionnaire_box_path, dpi=150)

print(f"\nSaved objective plot to {objective_path}")
print(f"Saved objective boxplot to {objective_box_path}")
print(f"Saved objective violin plot to {objective_violin_path}")
if learning_curve_path is not None:
    print(f"Saved learning-curve boxplot to {learning_curve_path}")
if gaze_heatmap_path is not None:
    print(f"Saved gaze group heatmaps to {gaze_heatmap_path}")
if gaze_metrics_box_path is not None:
    print(f"Saved gaze metrics boxplot to {gaze_metrics_box_path}")
if SHOW_QUESTIONNAIRE_PLOTS:
    print(f"Saved questionnaire plot to {questionnaire_path}")
    print(f"Saved questionnaire boxplot to {questionnaire_box_path}")
else:
    print("Questionnaire plots hidden.")
plt.show()
