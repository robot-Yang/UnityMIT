# === Averages limited to the Run window (start..stop) + robust end-state classification ===
import json, math, glob, os, sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon as MplPolygon
try:
    import tkinter as tk
    from tkinter import filedialog
except Exception:
    tk = None
# from matplotlib import cm

# ---------------- Config ----------------
REF_SCALE = 0.3
REF_STEPS = [
    (0, 140), (-140, 0), (0, 100), (100, 0),
    (0, 160), (-100, 0), (0, 100), (-140, 0),
    (0, -160), (-200, 0), (0, 100), (100, 0),
]

# ---------------- Coverage Config ----------------
SENSING_RADIUS = 0.25 #0.3         # [same units as trajectories] effective sensing radius per drone
EXCLUDE_COVERAGE_SEGMENTS = [0]  # list of 0-based segment indices to skip in coverage calculations

# Segment indices (0-based) in REF_STEPS
SEG_IDX_1 = 0   # first line of REF_STEPS
SEG_IDX_2 = 1   # second line
SEG_IDX_4 = 3   # fourth line

# Per-segment workspace widths in *world meters*, scaled by REF_SCALE to match ref_poly units
SEG_WIDTHS = {
    SEG_IDX_1: 14.0 * REF_SCALE,  # originally 16 m wide
    SEG_IDX_2: 18.0 * REF_SCALE,  # 18 m wide
    SEG_IDX_4: 22.0 * REF_SCALE,  # 22 m wide
}

# Trimming rules (meters along the segment, in world units, then scaled)
TRIM_20 = 20.0 * REF_SCALE
SEG_TRIMS = {
    SEG_IDX_1: dict(trim_start=0.0,     trim_end=TRIM_20),  # 20 m shorter from end
    SEG_IDX_2: dict(trim_start=TRIM_20, trim_end=TRIM_20),  # 20 m from start & end
    SEG_IDX_4: dict(trim_start=TRIM_20, trim_end=TRIM_20),  # 20 m from start & end
}

# Optional: shapely for geometric area computation (union of disks)
try:
    from shapely.geometry import Point, Polygon
    from shapely.ops import unary_union
except ImportError:
    Point = Polygon = unary_union = None
    print("WARNING: shapely is not installed; geometric area coverage "
          "will be skipped. Install via `pip install shapely` to enable it.",
          file=sys.stderr)

# Coverage subtraction: if True, coverage = area inside target − area outside target
SUBTRACT_OUTSIDE_COVERAGE = True
# SUBTRACT_OUTSIDE_COVERAGE = False
# 

def build_workspace_rect_and_poly(ref_poly, seg_index, width, trim_start=0.0, trim_end=0.0):
    """
    Build a rectangle of given width around a subsegment of the reference
    segment `seg_index`, aligned with it.

    The subsegment is obtained by trimming `trim_start` from the start
    and `trim_end` from the end (all in same units as ref_poly).

    Returns:
        coords_np: (5,2) closed polygon for plotting
        poly: shapely Polygon (or None if shapely missing)
        seg_len_effective: effective segment length after trimming
    """
    import numpy as _np
    a = _np.asarray(ref_poly[seg_index], dtype=float)
    b = _np.asarray(ref_poly[seg_index + 1], dtype=float)

    dx, dz = b - a
    seg_len = float(_np.hypot(dx, dz))
    if seg_len == 0.0:
        raise ValueError("Zero-length reference segment; cannot build workspace.")

    if trim_start + trim_end >= seg_len:
        raise ValueError(
            f"trim_start + trim_end = {trim_start+trim_end:.3f} >= segment length {seg_len:.3f}"
        )

    # unit direction along segment (from a to b)
    t_hat = _np.array([dx, dz], dtype=float) / seg_len
    a_short = a + t_hat * trim_start
    b_short = b - t_hat * trim_end

    dx_s, dz_s = b_short - a_short
    eff_seg_len = float(_np.hypot(dx_s, dz_s))

    # unit perpendicular (left-hand)
    n = _np.array([-dz_s, dx_s], dtype=float) / eff_seg_len
    half_w = width / 2.0

    c1 = a_short + n * half_w
    c2 = a_short - n * half_w
    c3 = b_short - n * half_w
    c4 = b_short + n * half_w

    coords = _np.vstack([c1, c2, c3, c4, c1])  # closed polygon
    poly = Polygon(coords) if Polygon is not None else None
    return coords, poly, eff_seg_len


def shapely_to_patches(geom, **patch_kwargs):
    """Convert a shapely (Multi)Polygon to a list of Matplotlib Polygon patches."""
    patches = []
    if geom is None or geom.is_empty:
        return patches
    try:
        from shapely.geometry import Polygon as ShapelyPolygon, MultiPolygon
    except ImportError:
        return patches

    if geom.geom_type == "Polygon":
        polys = [geom]
    elif geom.geom_type == "MultiPolygon":
        polys = list(geom.geoms)
    else:
        return patches

    for poly in polys:
        x, y = poly.exterior.xy
        coords = np.column_stack([x, y])
        patch = MplPolygon(coords, closed=True, **patch_kwargs)
        patches.append(patch)
    return patches


# How forgiving to be when deciding if a drone was "present at stop"
# (covers sampling quantization and scene-switch save slop).
SCENE_SWITCH_GRACE_S = 1.0

OUT_DIR = Path("outputs"); OUT_DIR.mkdir(parents=True, exist_ok=True)
OUT_TRAJ_PNG = str(OUT_DIR / "one_script_trajectories.png")
OUT_ERR_PNG  = str(OUT_DIR / "one_script_centroid_error.png")
OUT_INTERDIST_BOTH_PNG = str(OUT_DIR / "average_interagent_distance_both.png")
OUT_WIDTH_EMB_PNG = str(OUT_DIR / "swarm_width_embodied.png")
OUT_WIDTH_SEG_PNG = str(OUT_DIR / "swarm_width_segment.png")
OUT_SWEEP_METRICS = str(OUT_DIR / "swarm_sweep_coverage.txt")
DEFAULT_DATA_DIR = "/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default"
_FILE_PICKED = None  # avoid重复弹窗

def _get_cli_arg(flag):
    """Return the value following `flag` in sys.argv, if present."""
    if flag in sys.argv:
        idx = sys.argv.index(flag)
        if idx + 1 < len(sys.argv):
            return sys.argv[idx + 1]
    return None

CLI_INPUT_PATH = _get_cli_arg("--input") or _get_cli_arg("-i")
METRICS_ONLY = "--metrics-only" in sys.argv
VERBOSE = not METRICS_ONLY
FIG_OPTION_KEYS = ["traj", "cte", "interdist", "width_emb", "width_seg", "interactive"]


def pick_input_json(default_dir: str) -> Path:
    """
    Auto-pick the latest JSON under default_dir, with an option to choose another file.
    """
    global _FILE_PICKED
    if _FILE_PICKED is not None:
        return _FILE_PICKED
    base = Path(default_dir)
    files = sorted(base.rglob("*.json"), key=os.path.getmtime, reverse=True) if base.exists() else []
    if not files:
        raise FileNotFoundError(f"No JSON files found under {default_dir}")
    latest = files[0]
    print(f"\n最新文件(自动选择): {latest}")
    choice = input("回车使用它，输入路径，或输入 'b' 弹出文件选择窗口: ").strip()
    if choice == "":
        _FILE_PICKED = latest
        return latest
    if choice.lower() == "b":
        if tk is None:
            raise RuntimeError("tkinter 不可用，无法弹出文件选择窗口。")
        try:
            root = tk.Tk()
            root.withdraw()  # 隐藏 tk 主窗
            root.wm_attributes("-topmost", 1)
            root.update_idletasks()
            root.after(0, root.lift)
            root.after(0, root.focus_force)
            try:
                fp = filedialog.askopenfilename(
                    parent=root,
                    initialdir=default_dir,
                    filetypes=[("JSON","*.json")],
                    title="选择轨迹 JSON 文件"
                )
            finally:
                try:
                    root.destroy()
                except Exception:
                    pass
                try:
                    root.quit()
                except Exception:
                    pass
                if hasattr(tk, "_default_root"):
                    tk._default_root = None
            if fp:
                _FILE_PICKED = Path(fp)
                return _FILE_PICKED
            # 若未选择则继续用最新文件
            _FILE_PICKED = latest
            return latest
        except Exception as e:
            print(f"文件选择对话框失败: {e}")
            _FILE_PICKED = latest
            return latest
    # 否则把输入当作路径
    _FILE_PICKED = Path(choice)
    return _FILE_PICKED

def select_figures():
    opts = [
        ("traj", "Trajectories & centroid (static)"),
        ("cte", "Centroid error vs time"),
        ("interdist", "Inter-agent distance"),
        ("width_emb", "Width vs time (embodied forward)"),
        ("width_seg", "Width vs time (⟂ segment)"),
        ("interactive", "Interactive slider + coverage/sweep"),
    ]
    print("\n选择要生成的图（逗号分隔索引，留空=全部）：")
    for i, (_, desc) in enumerate(opts):
        print(f"  [{i}] {desc}")
    resp = input("选择: ").strip()
    if resp == "":
        return {k: True for k, _ in opts}
    sel = set(s.strip() for s in resp.split(",") if s.strip() != "")
    return {k: (str(i) in sel) for i, (k, _) in enumerate(opts)}

# Bring all Matplotlib figure windows to the front (best-effort, TkAgg friendly).
def _raise_all_figs():
    try:
        import matplotlib
        mgrs = matplotlib._pylab_helpers.Gcf.get_all_fig_managers()
        for m in mgrs:
            win = getattr(m, "window", None)
            if win is None:
                continue
            try:
                win.wm_attributes("-topmost", 1)
                win.lift()
                win.focus_force()
                win.update_idletasks()
                # allow other windows to take focus again after the lift
                win.after(200, lambda w=win: w.wm_attributes("-topmost", 0))
            except Exception:
                pass
    except Exception:
        pass

def _show_with_raise():
    """先创建窗口再强制置顶，然后进入阻塞式 show。"""
    try:
        plt.show(block=False)
        _raise_all_figs()
        plt.pause(0.2)
        plt.show()
    except Exception:
        plt.show()

# -------- Select input JSON (edit as needed) --------
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251031_232835_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251101_133056_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251101_205912_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251101_223938_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("..."), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251102_004342_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251102_005706_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Trajectories_1022/Setup_H_NO_20251102_010220_traj.json"), key=os.path.getmtime, reverse=True)

# shuhang without haptic
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang/Setup_H_NO_20251102_134927_traj.json"), key=os.path.getmtime, reverse=True)

# fuda with haptic
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Fuda/Setup_H_NO_20251102_150636_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Fuda/Setup_H_NO_20251102_150935_traj.json"), key=os.path.getmtime, reverse=True) 

# hongze haptic
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/hongze/Setup_H_NO_20251105_174008_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/hongze/Setup_H_NO_20251105_174259_traj.json"), key=os.path.getmtime, reverse=True)

# coverage test haptic
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251115_190518_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251115_195029_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251115_201319_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251115_201630_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251115_213503_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251115_214035_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251115_214704_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251115_215658_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251115_220025_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251116_000352_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251116_001006_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251116_121304_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251116_235954_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251117_000502_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251117_145754_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251117_150041_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251117_230430_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251117_230556_traj.json"), key=os.path.getmtime, reverse=True)

candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_145905_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_145801_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_153858_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_154049_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_154624_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_154752_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_155005_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_161600_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_161708_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_161815_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_161939_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_162034_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_162126_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_162216_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_221320_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_221723_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_222732_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251124_223312_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251125_115724_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251125_120350_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251125_120634_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251125_121349_traj.json"), key=os.path.getmtime, reverse=True)

# shuhang11_26 without haptic
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_26/Setup_H_NO_20251126_193152_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_26/Setup_H_NO_20251126_193448_traj.json"), key=os.path.getmtime, reverse=True)
# 
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_26/Setup_H_NO_20251126_201109_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_26/Setup_H_NO_20251126_201940_traj.json"), key=os.path.getmtime, reverse=True)

# test without haptic
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251126_214518_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251126_224338_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251126_223243_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251126_223755_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251126_232153_traj.json"), key=os.path.getmtime, reverse=True)

# new location
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_020502_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_021109_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_023808_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_024255_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_104912_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_115358_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_125553_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_125900_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_130936_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_131339_traj.json"), key=os.path.getmtime, reverse=True)

candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_140022_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_140340_traj.json"), key=os.path.getmtime, reverse=True)

candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_142649_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251127_144134_traj.json"), key=os.path.getmtime, reverse=True)

candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251129_214458_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251129_214959_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251129_215356_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251129_215621_traj.json"), key=os.path.getmtime, reverse=True)

candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251129_222126_traj.json"), key=os.path.getmtime, reverse=True)

candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251129_223626_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251130_005711_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251130_010006_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251130_130008_traj.json"), key=os.path.getmtime, reverse=True)

candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251130_131057_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251130_131736_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251130_171652_traj.json"), key=os.path.getmtime, reverse=True)
candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251130_171808_traj.json"), key=os.path.getmtime, reverse=True)

candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251130_174320_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/Shuhang11_29/Setup_H_NO_20251130_174439_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251130_181531_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251130_182238_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251130_182418_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251130_205232_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251130_222757_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251130_224058_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251130_224159_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251201_003922_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251201_004134_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251201_135709_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251201_140055_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251201_135825_traj.json"), key=os.path.getmtime, reverse=True)
# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251201_140318_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251201_165528_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("/Users/chenyang/ToGoogleDrive/Gitchen/UnityMIT/SoundMapping/SoundMappingUnity/Assets/Data/default/test/Setup_H_NO_20251201_165650_traj.json"), key=os.path.getmtime, reverse=True)

# candidates = sorted(glob.glob("..."), key=os.path.getmtime, reverse=True)

# -------- Select input JSON (auto-pick latest, option to browse) --------
if CLI_INPUT_PATH:
    INPUT_JSON = Path(CLI_INPUT_PATH)
else:
    INPUT_JSON = pick_input_json(DEFAULT_DATA_DIR)

if not INPUT_JSON.exists():
    raise FileNotFoundError(f"Input JSON not found: {INPUT_JSON}")

FIG_FLAGS = {k: False for k in FIG_OPTION_KEYS} if METRICS_ONLY else select_figures()

with INPUT_JSON.open("r") as f:
    data = json.load(f)

scene = data.get("scene", data.get("level", "Unknown Scene"))
sample_hz = data.get("sampleHz", None)
# If your file didn’t record sampleHz, set a fallback here:
if not isinstance(sample_hz, (int, float)) or sample_hz <= 0:
    sample_hz = 5.0

# ---- Embodied metadata from file (optional, written by recorder) ----
def _clean_embodied_id(val):
    # Recorder may write -2147483648 when unknown.
    if isinstance(val, int) and val != -2147483648:
        return val
    return None

embodied_id_meta   = _clean_embodied_id(data.get("embodiedId"))
embodied_name_meta = data.get("embodiedName") or None
if embodied_name_meta == "": embodied_name_meta = None

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
        e_arr = [fr.get("e", None) for fr in frames]
        qx_arr = [fr.get("qx", None) for fr in frames]
        qy_arr = [fr.get("qy", None) for fr in frames]
        qz_arr = [fr.get("qz", None) for fr in frames]
        qw_arr = [fr.get("qw", None) for fr in frames]
        drone_tracks[name] = {
            "id": traj.get("id", None),
            "embodied": bool(traj.get("embodied", False)),
            "t": np.array(t_arr, dtype=float) if (t_arr and t_arr[0] is not None) else None,
            "x": np.array(x_arr, dtype=float),
            "z": np.array(z_arr, dtype=float),
            "g": np.array(g_arr, dtype=float) if any(v is not None for v in g_arr) else None,
            "e": np.array(e_arr, dtype=float) if any(v is not None for v in e_arr) else None,
            "q": np.column_stack([qx_arr, qy_arr, qz_arr, qw_arr]).astype(float) if any(v is not None for v in qx_arr) else None,
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
        e_arr = [p.get("e", None) for p in pos]
        qx_arr = [p.get("qx", None) for p in pos]
        qy_arr = [p.get("qy", None) for p in pos]
        qz_arr = [p.get("qz", None) for p in pos]
        qw_arr = [p.get("qw", None) for p in pos]
        t_here = top_time if (top_time is not None and len(top_time) == len(x_arr)) else None
        drone_tracks[name] = {
            "id": entry.get("droneId", None),
            "embodied": bool(entry.get("embodied", False)),
            "t": t_here,
            "x": np.array(x_arr, dtype=float),
            "z": np.array(z_arr, dtype=float),
            "g": np.array(g_arr, dtype=float) if any(v is not None for v in g_arr) else None,
            "e": np.array(e_arr, dtype=float) if any(v is not None for v in e_arr) else None,
            "q": np.column_stack([qx_arr, qy_arr, qz_arr, qw_arr]).astype(float) if any(v is not None for v in qx_arr) else None,
        }
else:
    raise ValueError("Unrecognized JSON layout (expected 'trajectories' or 'swarmState').")
if not drone_tracks:
    raise ValueError("No drone trajectories found.")

# Helper: decide if a track is the embodied drone
def is_embodied_track(name, track, embodied_id_meta, embodied_name_meta):
    if track.get("embodied"):
        return True
    tid = track.get("id", None)
    if embodied_id_meta is not None and tid is not None and tid == embodied_id_meta:
        return True
    if embodied_name_meta is not None and name == embodied_name_meta:
        return True
    return False

def embodied_name_timeline(times_arr, tracks, use_time, sample_hz):
    """
    Return list aligned to times_arr with the embodied drone name at each sample
    based on per-frame 'e' flags. None if unknown for that time.
    """
    if times_arr is None or len(times_arr) == 0:
        return []

    # Build time->name mapping from per-frame e flags
    e_lookup = {}
    for name, tr in tracks.items():
        e = tr.get("e")
        if e is None or not len(e):
            continue
        t_arr = tr.get("t")
        if t_arr is None:
            # use index as time if no real time
            t_arr = np.arange(len(e), dtype=float) / float(sample_hz) if sample_hz else np.arange(len(e), dtype=float)
        for ti, ei in zip(t_arr, e):
            if ei == 1 or ei == 1.0:
                e_lookup[round(float(ti), 3)] = name

    result = []
    for t in times_arr:
        result.append(e_lookup.get(round(float(t), 3)))
    return result

def _contiguous_true_spans(mask):
    spans = []
    if mask.size == 0:
        return spans
    start = None
    for i, val in enumerate(mask):
        if val and start is None:
            start = i
        if (not val or i == len(mask)-1) and start is not None:
            end = i if val else i-1
            spans.append((start, end))
            start = None
    return spans

def plot_embodied_segments(ax, track, color, linewidth=3.2, alpha=0.95):
    """Overlay thick segments where e==1."""
    e = track.get("e")
    if e is None or not len(e):
        return
    xs = np.asarray(track["x"], dtype=float)
    zs = np.asarray(track["z"], dtype=float)
    mask = np.asarray(e, dtype=float) >= 0.5
    spans = _contiguous_true_spans(mask)
    for s, e_idx in spans:
        ax.plot(xs[s:e_idx+1], zs[s:e_idx+1], linewidth=linewidth, alpha=alpha, color=color, zorder=4)

# -------- Reference path --------
pts = [(0.0, 50.0)] # start point of reference path
x, z = 0.0, 0.0
for dx, dz in REF_STEPS:
    x += dx; z += dz
    pts.append((x, z))
ref_poly = np.array(pts, dtype=float) * REF_SCALE

# --- Workspace rectangles for segments 1, 2, and 4 ---
rects_coords = {}
workspace_polys = {}
workspace_lengths = {}
workspace_areas = {}

for seg_idx in [SEG_IDX_1, SEG_IDX_2, SEG_IDX_4]:
    trims = SEG_TRIMS[seg_idx]
    width = SEG_WIDTHS[seg_idx]
    coords, poly, eff_len = build_workspace_rect_and_poly(
        ref_poly, seg_idx,
        width,
        trim_start=trims["trim_start"],
        trim_end=trims["trim_end"],
    )
    rects_coords[seg_idx] = coords
    workspace_polys[seg_idx] = poly
    workspace_lengths[seg_idx] = eff_len
    workspace_areas[seg_idx] = eff_len * width

# For convenience: first coverage window (segment 1)
rect_seg0_coords    = rects_coords[SEG_IDX_1]
workspace_poly_seg0 = workspace_polys[SEG_IDX_1]
seg0_length         = workspace_lengths[SEG_IDX_1]
workspace_area_seg0 = workspace_areas[SEG_IDX_1]

# ---- Nearest-segment helpers (track which segment is closest) ----
def closest_point_on_segment_with_idx(p, a, b, seg_idx):
    ap = p - a; ab = b - a
    ab2 = float(ab[0]*ab[0] + ab[1]*ab[1])
    if ab2 == 0.0:
        q = a
        d2 = float((p[0]-a[0])**2 + (p[1]-a[1])**2)
        return math.sqrt(d2), seg_idx
    t = (ap[0]*ab[0] + ap[1]*ab[1]) / ab2
    t = max(0.0, min(1.0, t))
    q = a + t*ab
    d2 = float((p[0]-q[0])**2 + (p[1]-q[1])**2)
    return math.sqrt(d2), seg_idx

def dist_point_to_polyline_with_segidx(p, poly):
    best_d = float("inf")
    best_idx = -1
    for i in range(len(poly)-1):
        d, _ = closest_point_on_segment_with_idx(p, poly[i], poly[i+1], i)
        if d < best_d:
            best_d = d
            best_idx = i
    return best_d, best_idx

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

# Compute centroid error + nearest segment index
centroid_err_list = []
centroid_segidx_list = []
for k in range(len(centroid)):
    d, seg_idx = dist_point_to_polyline_with_segidx(centroid[k], ref_poly)
    centroid_err_list.append(d)
    centroid_segidx_list.append(seg_idx)
centroid_err = np.array(centroid_err_list, dtype=float)
centroid_segidx = np.array(centroid_segidx_list, dtype=int)

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
disconnected_end = sum(1 for s in per_drone_status.values() if s['status']=='disconnected_at_end')
crashed_early = sum(1 for s in per_drone_status.values() if s['status']=='crashed_or_vanished_early')
crashed_disconnected = sum(1 for s in per_drone_status.values() if s['status']=='vanished_while_disconnected')
crashed_total = crashed_early + crashed_disconnected
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

# Average centroid→reference distance (run mask) with optional segment exclusion.
# - Set EXCLUDE_SEG_INDICES to None or an empty iterable to include all segments.
# - Set to an int or iterable of 0-based indices to exclude (e.g., [2] to drop segment #3).
EXCLUDE_SEG_INDICES = []

def _exclude_mask(seg_idx_arr, exclude_cfg):
    """Return boolean mask of samples to drop based on nearest-segment indices."""
    exclude_set = _normalize_exclude(exclude_cfg)
    if not exclude_set:
        return np.zeros_like(seg_idx_arr, dtype=bool)
    return np.isin(seg_idx_arr, list(exclude_set))

def _normalize_exclude(exclude_cfg):
    """Normalize exclude config into a set of int indices."""
    if exclude_cfg is None:
        return set()
    if isinstance(exclude_cfg, (int, np.integer)):
        return {int(exclude_cfg)}
    return {int(x) for x in exclude_cfg}

def _quat_forward_right(q):
    """Return forward (x,z) and right (x,z) unit vectors from quaternion [x,y,z,w]."""
    if q is None or len(q) != 4:
        return None, None
    qx, qy, qz, qw = [float(v) for v in q]
    # Forward = rotation of (0,0,1)
    fx = 2.0 * (qx*qz + qw*qy)
    fz = 1.0 - 2.0 * (qx*qx + qy*qy)
    norm = math.hypot(fx, fz)
    if norm <= 1e-6:
        return None, None
    fx /= norm; fz /= norm
    rx, rz = fz, -fx  # right perpendicular in XZ plane
    return (fx, fz), (rx, rz)

def _build_embodied_pose_lookup(tracks, use_time, sample_hz):
    """
    Build lookup: rounded time -> (pos(x,z), quat(x,y,z,w))
    using frames where e==1.
    """
    lookup = {}
    for name, tr in tracks.items():
        e = tr.get("e")
        q = tr.get("q")
        if e is None or q is None or len(e) == 0 or q.shape[0] == 0:
            continue
        ts = tr.get("t")
        if ts is None:
            n = len(e)
            ts = np.arange(n, dtype=float) / float(sample_hz) if sample_hz else np.arange(n, dtype=float)
        for xi, zi, ei, qi, ti in zip(tr["x"], tr["z"], e, q, ts):
            if ei >= 0.5:
                lookup[round(float(ti), 3)] = ((float(xi), float(zi)), qi)
    return lookup

if np.any(mask_cte):
    exclude_set = _normalize_exclude(EXCLUDE_SEG_INDICES)
    mask_exclude = _exclude_mask(centroid_segidx, exclude_set)
    mask_for_avg = mask_cte & ~mask_exclude
    avg_err_m = float(np.mean(centroid_err[mask_for_avg])) if np.any(mask_for_avg) else float("nan")
    excluded = int(np.sum(mask_cte & mask_exclude))
    total_in_run = int(np.sum(mask_cte))
else:
    exclude_set = set()
    avg_err_m = float("nan")
    excluded = 0
    total_in_run = 0

# -------- Swarm width (main group, relative to embodied orientation) --------
emb_pose_lookup = _build_embodied_pose_lookup(drone_tracks, use_time, sample_hz)

def _swarm_width_embodied(time_key, pts):
    """Compute lateral width of pts wrt embodied look direction at time_key (XZ plane)."""
    pose = emb_pose_lookup.get(round(float(time_key), 3))
    if pose is None:
        return float("nan")
    (ex, ez), q = pose
    fwd, right = _quat_forward_right(q)
    if right is None or not pts:
        return float("nan")
    projections = []
    for (x, z) in pts:
        dx, dz = x - ex, z - ez
        proj = dx * right[0] + dz * right[1]
        projections.append(proj)
    return float(max(projections) - min(projections)) if projections else float("nan")

def _swarm_width_perp_segment(idx, pts):
    """
    Width of main group at sample idx, measured perpendicular to the nearest
    reference segment's direction.
    """
    if idx < 0 or idx >= len(centroid):
        return float("nan")
    seg_idx = int(centroid_segidx[idx])
    if seg_idx < 0 or seg_idx >= (len(ref_poly) - 1) or not pts:
        return float("nan")
    a = np.asarray(ref_poly[seg_idx], dtype=float)
    b = np.asarray(ref_poly[seg_idx + 1], dtype=float)
    ab = b - a
    seg_len = float(np.hypot(ab[0], ab[1]))
    if seg_len <= 0.0:
        return float("nan")
    t_hat = ab / seg_len
    n_hat = np.array([-t_hat[1], t_hat[0]])
    c = np.asarray(centroid[idx], dtype=float)
    w_coords = []
    for (x, z) in pts:
        d = np.array([x, z]) - c
        w_coords.append(float(d[0] * n_hat[0] + d[1] * n_hat[1]))
    if not w_coords:
        return float("nan")
    return float(max(w_coords) - min(w_coords))

# Width vs time (embodied frame)
swarm_width_times_emb = []
swarm_width_vals_emb = []

# Width vs time (perpendicular to nearest segment)
swarm_width_times_seg = []
swarm_width_vals_seg = []

if use_time:
    for idx, t_key in enumerate(times):
        pts_here = bins_all.get(round(float(t_key), 3), [])
        # Embodied-based width
        w_emb = _swarm_width_embodied(t_key, pts_here) if pts_here else float("nan")
        swarm_width_times_emb.append(float(t_key))
        swarm_width_vals_emb.append(w_emb)
        # Segment-perpendicular width
        w_seg = _swarm_width_perp_segment(idx, pts_here) if pts_here else float("nan")
        swarm_width_times_seg.append(float(t_key))
        swarm_width_vals_seg.append(w_seg)
else:
    min_len = min(len(d["x"]) for d in drone_tracks.values())
    ts = np.arange(min_len, dtype=float)
    for idx in range(min_len):
        pts_here = []
        for d in drone_tracks.values():
            g = d.get("g", None)
            if g is not None and not (g[idx] == 1):
                continue
            pts_here.append((float(d["x"][idx]), float(d["z"][idx])))
        if not pts_here:
            pts_here = [(float(d["x"][idx]), float(d["z"][idx])) for d in drone_tracks.values()]
        t_here = ts[idx] / float(sample_hz) if sample_hz else ts[idx]
        w_emb = _swarm_width_embodied(t_here, pts_here)
        w_seg = _swarm_width_perp_segment(idx, pts_here)
        swarm_width_times_emb.append(ts[idx])
        swarm_width_vals_emb.append(w_emb)
        swarm_width_times_seg.append(ts[idx])
        swarm_width_vals_seg.append(w_seg)

# Lookup for sweep calculations (use segment-perpendicular width over time)
width_lookup = {round(float(t), 3): float(w) for t, w in zip(swarm_width_times_seg, swarm_width_vals_seg)}

# Run total spent time
run_total_spent_time_s = float(t1 - t0) if (t0 is not None and t1 is not None) else float(total_time_s)

# Average inter-agent distances (run masks)
avg_interagent_main_overall  = float(np.nanmean(avg_interagent[mask_inter_main]))   if avg_interagent.size else float("nan")
avg_interagent_swarm_overall = float(np.nanmean(avg_interagent_swarm[mask_inter_swarm])) if avg_interagent_swarm.size else float("nan")

# ---------- Midpoint-line crossing for a given reference segment ----------
def _segment_endpoints(poly, seg_idx_1based):
    i = int(seg_idx_1based) - 1  # convert to 0-based
    if i < 0 or i >= (len(poly) - 1):
        raise IndexError(f"Segment {seg_idx_1based} out of range (valid 1..{len(poly)-1})")
    a = np.asarray(poly[i], dtype=float)      # [x, z]
    b = np.asarray(poly[i+1], dtype=float)    # [x, z]
    ab = b - a
    L2 = float(ab[0]*ab[0] + ab[1]*ab[1])
    if L2 == 0.0:
        raise ValueError("Zero-length segment.")
    L = math.sqrt(L2)
    t_hat = ab / L                            # unit direction along segment
    # n_hat = np.array([-t_hat[1], t_hat[0]])   # unit perpendicular (left-hand)
    n_hat = np.array([t_hat[1], -t_hat[0]])   # unit perpendicular
    if VERBOSE:
        print (f"Segment {seg_idx_1based}: a={a}, b={b}, L={L}, t̂={t_hat}, n̂={n_hat}")
    m = 0.5 * (a + b)                         # midpoint
    return a, b, ab, L, L2, t_hat, n_hat, m

def _time_array_for_track(track, sample_hz):
    if track.get("t") is not None and len(track["t"]) > 0:
        return np.asarray(track["t"], dtype=float)
    n = len(track.get("x", []))
    return np.arange(n, dtype=float) / float(sample_hz)

def count_midline_crossings_by_side(seg_idx_1based=10, eps=1e-6):
    """
    For each drone, detect the FIRST crossing of the midpoint line of the given segment.
    Classify by:
      - side_at_cross: 'up' if (P_cross - a)·n̂ > 0, 'down' if < 0   (side of segment line at crossing)
      - from_side:     side sign at the sample just BEFORE the crossing (approach side)

    Returns:
      {
        'segment_1based': int,
        'upper_count': int, 'down_count': int,
        'from_upper_count': int, 'from_down_count': int,
        'split_metric': int,               # based on upper/down at the crossing
        'split_metric_approach': int,      # based on approach side
        'per_drone': [(name, t_cross, side_at_cross, from_side, n_cross), ...]
      }
    """
    a, b, ab, L, L2, t_hat, n_hat, M = _segment_endpoints(ref_poly, seg_idx_1based)

    per_drone = []
    upper_count = down_count = 0
    from_upper_count = from_down_count = 0

    for name, tr in drone_tracks.items():
        xs = np.asarray(tr.get("x", []), dtype=float)
        zs = np.asarray(tr.get("z", []), dtype=float)
        if xs.size < 2:
            continue

        ts = _time_array_for_track(tr, sample_hz)
        P  = np.column_stack([xs, zs])            # (N,2)

        # s = signed coord along segment relative to midpoint (midline is s==0)
        s  = (P - M) @ t_hat
        s0 = s[:-1]
        s1 = s[ 1:]

        # robust zero tolerance
        s0c = np.where(np.abs(s0) <= eps, 0.0, s0)
        s1c = np.where(np.abs(s1) <= eps, 0.0, s1)

        # crossing if sign flips or touches zero between samples
        cross_mask = (s0c * s1c < 0.0) | ((s0c == 0.0) & (s1c != 0.0)) | ((s0c != 0.0) & (s1c == 0.0))
        if not np.any(cross_mask):
            continue

        k = int(np.argmax(cross_mask))  # first crossing index (between k and k+1)

        # interpolation fraction of zero of s between k and k+1
        denom = abs(s0[k]) + abs(s1[k])
        alpha = 0.0 if denom == 0.0 else (abs(s0[k]) / denom)

        # crossing time and position
        t0, t1 = ts[k], ts[k+1]
        t_cross = float(t0 + (t1 - t0) * alpha)
        P_cross = P[k] + alpha * (P[k+1] - P[k])

        # side of segment at crossing and just before (using perpendicular n̂)
        n_cross  = float((P_cross - a) @ n_hat)
        n_before = float((P[k]    - a) @ n_hat)

        side_at_cross = 'up'   if n_cross  > +eps else ('down' if n_cross  < -eps else 'on')
        from_side     = 'up'   if n_before > +eps else ('down' if n_before < -eps else 'on')

        if side_at_cross == 'up':   upper_count += 1
        elif side_at_cross == 'down': down_count += 1

        if from_side == 'up':     from_upper_count += 1
        elif from_side == 'down': from_down_count += 1

        per_drone.append((name, t_cross, side_at_cross, from_side, n_cross))

    per_drone.sort(key=lambda x: x[1])

    # --- Split metrics ---
    # Your example implies using floor(|diff|/2) to measure how many would have to switch
    # to get as even as possible.
    split_metric = int(abs(upper_count - down_count) // 2)
    split_metric_approach = int(abs(from_upper_count - from_down_count) // 2)

    return {
        'segment_1based': seg_idx_1based,
        'upper_count': upper_count,
        'down_count': down_count,
        'from_upper_count': from_upper_count,
        'from_down_count': from_down_count,
        'split_metric': split_metric,
        'split_metric_approach': split_metric_approach,
        'per_drone': per_drone
    }


res = count_midline_crossings_by_side(seg_idx_1based=10, eps=1e-6)
if VERBOSE:
    print(f"\n[Midpoint-line crossings @ segment {res['segment_1based']}]")
    print(f"  Crossed ON upper side: {res['upper_count']}")
    print(f"  Crossed ON down  side: {res['down_count']}")
    print(f"  Came FROM upper side (approach): {res['from_upper_count']}")
    print(f"  Came FROM down  side (approach): {res['from_down_count']}")
    print(f"  Split metric (at crossing): {res['split_metric']}   "
          f"[upper={res['upper_count']}, down={res['down_count']}]")
    print(f"  Split metric (approach):    {res['split_metric_approach']}   "
          f"[from_upper={res['from_upper_count']}, from_down={res['from_down_count']}]")
# for name, tc, side_at, from_side, nval in res['per_drone']:
#     print(f"    {name:>12s}  t={tc:.3f}s  at={side_at:>4s}  from={from_side:>4s}  n={nval:+.3f}")


# -------- Geometric area coverage helpers --------
def compute_segment_coverage(seg_index, workspace_poly, workspace_area):
    """
    Compute coverage for one segment:
      - union of disks for main-group drones
      - restricted to Run window
      - only when centroid is closest to this segment
      - then intersected with workspace_poly
      - optionally subtract area outside workspace (still covered by disks)
      - outside here is defined as spill perpendicular to the segment (width overflow),
        not along the segment.
    Returns:
      (union_poly_inside, covered_area_inside, coverage_pct_effective, union_poly_outside, covered_area_outside)
    """
    if Polygon is None or Point is None or unary_union is None:
        return None, 0.0, float("nan"), None, 0.0

    # Build a wide strip aligned with the segment to measure perpendicular spillover only.
    a = np.asarray(ref_poly[seg_index], dtype=float)
    b = np.asarray(ref_poly[seg_index + 1], dtype=float)
    ab = b - a
    seg_len = float(np.hypot(ab[0], ab[1]))
    if seg_len <= 0:
        return None, 0.0, float("nan"), None, 0.0
    t_hat = ab / seg_len
    n_hat = np.array([-t_hat[1], t_hat[0]])
    half_w = SEG_WIDTHS[seg_index] / 2.0
    trim_start = SEG_TRIMS[seg_index].get("trim_start", 0.0)
    trim_end   = SEG_TRIMS[seg_index].get("trim_end", 0.0)
    eff_len = max(seg_len - trim_start - trim_end, 0.0)

    # Build strip only over the trimmed workspace portion (plus a tiny buffer).
    seg_start_trim = a + t_hat * trim_start
    seg_end_trim   = b - t_hat * trim_end
    strip_len = eff_len + 2.0 * SENSING_RADIUS
    p_mid = 0.5 * (seg_start_trim + seg_end_trim)
    p0 = p_mid - t_hat * (strip_len / 2.0)
    p1 = p_mid + t_hat * (strip_len / 2.0)
    s1 = p0 + n_hat * half_w
    s2 = p0 - n_hat * half_w
    s3 = p1 - n_hat * half_w
    s4 = p1 + n_hat * half_w
    strip_poly = Polygon([s1, s2, s3, s4])

    positions = []

    if use_time:
        # bins_all: time_key -> list[(x,z)] for main group (g == 1)
        time_to_idx = {float(t): i for i, t in enumerate(times)}
        for t_key, pts_here in bins_all.items():
            idx = time_to_idx.get(float(t_key))
            if idx is None:
                continue
            # Only times in Run window AND whose centroid is closest to this segment
            if not mask_cte[idx]:
                continue
            if centroid_segidx[idx] != seg_index:
                continue
            positions.extend(pts_here)
    else:
        min_len = min(len(d["x"]) for d in drone_tracks.values())
        for idx in range(min_len):
            if not mask_cte[idx]:
                continue
            if centroid_segidx[idx] != seg_index:
                continue

            pts_here = []
            for d in drone_tracks.values():
                g = d.get("g", None)
                if g is not None and not (g[idx] == 1):
                    continue
                pts_here.append((float(d["x"][idx]), float(d["z"][idx])))

            if not pts_here:
                pts_here = [(float(d["x"][idx]), float(d["z"][idx])) for d in drone_tracks.values()]
            positions.extend(pts_here)

    if not positions:
        return None, 0.0, 0.0, None, 0.0

    disks_inside = []
    disks_outside = []
    for (x_p, z_p) in positions:
        disk_raw = Point(x_p, z_p).buffer(SENSING_RADIUS, resolution=32)
        inside_piece = disk_raw.intersection(workspace_poly) if workspace_poly is not None else disk_raw
        # Only subtract lateral spill if projection lies within the segment span (adjacent to workspace).
        outside_perp = None
        if SUBTRACT_OUTSIDE_COVERAGE:
            vec = np.array([x_p, z_p]) - a
            s_along = float(vec @ t_hat)
            if (trim_start <= s_along) and (s_along <= (seg_len - trim_end)):
                outside_perp = disk_raw.difference(strip_poly)

        if inside_piece and (not inside_piece.is_empty):
            disks_inside.append(inside_piece)
        if outside_perp and (not outside_perp.is_empty):
            disks_outside.append(outside_perp)

    if not disks_inside and not disks_outside:
        return None, 0.0, 0.0, None, 0.0

    union_inside = unary_union(disks_inside) if disks_inside else None
    union_outside = unary_union(disks_outside) if disks_outside else None

    area_inside = float(union_inside.area) if union_inside is not None else 0.0
    area_outside = float(union_outside.area) if (union_outside is not None and SUBTRACT_OUTSIDE_COVERAGE) else 0.0

    effective_area = max(area_inside - area_outside, 0.0) if SUBTRACT_OUTSIDE_COVERAGE else area_inside
    coverage_pct = 100.0 * effective_area / float(workspace_area) if workspace_area > 0 else float("nan")

    return union_inside, effective_area, coverage_pct, union_outside, area_outside

def compute_segment_coverage_if_needed(seg_index, workspace_poly, workspace_area):
    if seg_index in set(EXCLUDE_COVERAGE_SEGMENTS):
        return None, 0.0, float("nan"), None, 0.0, True
    cov = compute_segment_coverage(seg_index, workspace_poly, workspace_area)
    return (*cov, False)

def compute_sweep_coverage(seg_index, workspace_poly, workspace_area):
    """
    Coverage of the swarm treated as a whole: when the centroid is within the
    trimmed along-segment span. Swarm footprint per sample is modeled as a
    ribbon following the centroid along the reference segment, with width
    given by the main-group spread perpendicular to that segment.
    """
    if Polygon is None or Point is None or unary_union is None or workspace_poly is None:
        return None, 0.0, float("nan"), None

    # Segment geometry
    a = np.asarray(ref_poly[seg_index], dtype=float)
    b = np.asarray(ref_poly[seg_index + 1], dtype=float)
    ab = b - a
    seg_len = float(np.hypot(ab[0], ab[1]))
    if seg_len <= 0:
        return None, 0.0, float("nan"), None
    t_hat = ab / seg_len
    n_hat = np.array([-t_hat[1], t_hat[0]])
    trim_cfg = SEG_TRIMS.get(seg_index, dict(trim_start=0.0, trim_end=0.0))
    trim_start = float(trim_cfg.get("trim_start", 0.0))
    trim_end   = float(trim_cfg.get("trim_end", 0.0))
    eff_len = max(seg_len - trim_start - trim_end, 0.0)

    # Segment strip (for limiting perpendicular spread and per-segment isolation)
    half_w_strip = SEG_WIDTHS[seg_index] / 2.0
    seg_start_trim = a + t_hat * trim_start
    seg_end_trim   = b - t_hat * trim_end
    strip_len = eff_len + 2.0 * SENSING_RADIUS
    p_mid_strip = 0.5 * (seg_start_trim + seg_end_trim)
    p0_strip = p_mid_strip - t_hat * (strip_len / 2.0)
    p1_strip = p_mid_strip + t_hat * (strip_len / 2.0)
    strip_poly = Polygon([
        p0_strip + n_hat * half_w_strip,
        p0_strip - n_hat * half_w_strip,
        p1_strip - n_hat * half_w_strip,
        p1_strip + n_hat * half_w_strip,
    ])

    # Collect time indices where centroid is (a) in the Run window, (b) closest to this segment,
    # and (c) projected along this segment within the trimmed span.
    # For each such sample we store (centroid_center, half_width_perp_segment).
    sample_data = []
    if use_time:
        for i, c in enumerate(centroid):
            if not mask_cte[i]:
                continue
            if centroid_segidx[i] != seg_index:
                continue
            vec = c - a
            s_along = float(vec @ t_hat)
            if (s_along < trim_start) or (s_along > (seg_len - trim_end)):
                continue

            t_key = round(float(times[i]), 3)
            pts_here = bins_all.get(t_key, [])
            if not pts_here:
                continue

            # Use真实质心作为带的中心，但宽度方向仍然用段法向 n_hat。
            c_center = np.asarray(centroid[i], dtype=float)
            # Width = spread of main-group points along the segment normal around centroid
            w_coords = []
            for (x_p, z_p) in pts_here:
                d = np.array([x_p, z_p]) - c_center
                w_coords.append(float(d[0] * n_hat[0] + d[1] * n_hat[1]))
            if not w_coords:
                continue
            # Base positional spread along segment normal
            width_here = max(w_coords) - min(w_coords)
            # Inflate by sensing radius on both sides to match coverage footprint
            width_here = max(width_here + 2.0 * SENSING_RADIUS, 2.0 * SENSING_RADIUS)
            sample_data.append((c_center, width_here * 0.5))
    else:
        min_len = min(len(d["x"]) for d in drone_tracks.values())
        for i, c in enumerate(centroid[:min_len]):
            if not mask_cte[i]:
                continue
            if centroid_segidx[i] != seg_index:
                continue
            vec = c - a
            s_along = float(vec @ t_hat)
            if (s_along < trim_start) or (s_along > (seg_len - trim_end)):
                continue

            pts_here = []
            for d in drone_tracks.values():
                g = d.get("g", None)
                if g is not None and not (g[i] == 1):
                    continue
                pts_here.append((float(d["x"][i]), float(d["z"][i])))
            if not pts_here:
                pts_here = [(float(d["x"][i]), float(d["z"][i])) for d in drone_tracks.values()]

            c_center = np.asarray(centroid[i], dtype=float)
            w_coords = []
            for (x_p, z_p) in pts_here:
                d = np.array([x_p, z_p]) - c_center
                w_coords.append(float(d[0] * n_hat[0] + d[1] * n_hat[1]))
            if not w_coords:
                continue
            width_here = max(w_coords) - min(w_coords)
            width_here = max(width_here + 2.0 * SENSING_RADIUS, 2.0 * SENSING_RADIUS)
            sample_data.append((c_center, width_here * 0.5))

    if not sample_data:
        return None, 0.0, 0.0, None

    # Build a ribbon from consecutive centroid samples along the segment.
    # Rectangles are centered at the (真实)质心位置，方向用该段的切向/法向。
    rects = []
    # Sort by along-segment coordinate so the ribbon follows the walkway
    def _s_along(c_center):
        return float((np.asarray(c_center, dtype=float) - a) @ t_hat)
    sample_data.sort(key=lambda item: _s_along(item[0]))

    for idx in range(len(sample_data) - 1):
        c0, hw0 = sample_data[idx]
        c1, hw1 = sample_data[idx + 1]
        # 用两帧的平均半宽，避免过度“取最大”导致宽度看起来不变
        hw = max(0.5 * (hw0 + hw1), SENSING_RADIUS)
        c0 = np.asarray(c0, dtype=float)
        c1 = np.asarray(c1, dtype=float)
        p0 = c0 + n_hat * hw
        p1 = c0 - n_hat * hw
        p2 = c1 - n_hat * hw
        p3 = c1 + n_hat * hw
        rects.append(Polygon([p0, p1, p2, p3]))

    # Optional small caps at endpoints so the ribbon isn't hollow at start/end
    if sample_data:
        for c_mid, hw in (sample_data[0:1] + sample_data[-1:]):
            hw = max(hw, SENSING_RADIUS)
            c_mid = np.asarray(c_mid, dtype=float)
            cap_len = SENSING_RADIUS
            q0 = c_mid - t_hat * (cap_len * 0.5)
            q1 = c_mid + t_hat * (cap_len * 0.5)
            p0 = q0 + n_hat * hw
            p1 = q0 - n_hat * hw
            p2 = q1 - n_hat * hw
            p3 = q1 + n_hat * hw
            rects.append(Polygon([p0, p1, p2, p3]))

    if not rects:
        return None, 0.0, 0.0, None

    # Full sweep ribbon: 只限制在这条线段的长度范围（通过 sample 选择和矩形构造实现），
    # 不再用 strip_poly 在宽度方向上裁剪，这样宽度上超出 workspace 的部分也会保留。
    ribbon = unary_union(rects)

    # Inside = ribbon within workspace; outside = ribbon in strip but outside workspace
    inside_poly = ribbon.intersection(workspace_poly)
    outside_poly = ribbon.difference(workspace_poly) if SUBTRACT_OUTSIDE_COVERAGE else None

    area_inside = float(inside_poly.area) if inside_poly is not None and not inside_poly.is_empty else 0.0
    area_outside = float(outside_poly.area) if outside_poly is not None and not outside_poly.is_empty else 0.0
    effective = max(area_inside - area_outside, 0.0) if SUBTRACT_OUTSIDE_COVERAGE else area_inside
    coverage_pct = 100.0 * effective / float(workspace_area) if workspace_area > 0 else float("nan")
    # Return inside, outside, and the full ribbon (for visualization of total sweep).
    return inside_poly, area_inside, area_outside, effective, coverage_pct, outside_poly, ribbon

# Formatting / safety helpers (declared early so they are available for later calculations)
def _fmt(v):
    try:
        fv = float(v)
    except Exception:
        return "nan"
    return "nan" if math.isnan(fv) else f"{fv:.3f}"

def _sweep_get(tup, idx, default=float("nan")):
    if tup is None or not hasattr(tup, "__len__"):
        return default
    if idx >= len(tup):
        return default
    try:
        return float(tup[idx]) if tup[idx] is not None else default
    except Exception:
        return default

def _sweep_poly(tup, idx):
    try:
        return tup[idx] if (tup is not None and len(tup) > idx) else None
    except Exception:
        return None

# Compute coverage for three segments: 1, 2, and 4
coverage_poly_seg1, covered_area_seg1, coverage_pct_seg1, coverage_out_seg1, covered_out_area_seg1, seg1_excluded = compute_segment_coverage_if_needed(
    SEG_IDX_1, workspace_polys[SEG_IDX_1], workspace_areas[SEG_IDX_1]
)
coverage_poly_seg2, covered_area_seg2, coverage_pct_seg2, coverage_out_seg2, covered_out_area_seg2, seg2_excluded = compute_segment_coverage_if_needed(
    SEG_IDX_2, workspace_polys[SEG_IDX_2], workspace_areas[SEG_IDX_2]
)
coverage_poly_seg4, covered_area_seg4, coverage_pct_seg4, coverage_out_seg4, covered_out_area_seg4, seg4_excluded = compute_segment_coverage_if_needed(
    SEG_IDX_4, workspace_polys[SEG_IDX_4], workspace_areas[SEG_IDX_4]
)
# Pure inside areas (不扣 outside)
inside_area_seg1 = float(coverage_poly_seg1.area) if (coverage_poly_seg1 is not None and not seg1_excluded) else 0.0
inside_area_seg2 = float(coverage_poly_seg2.area) if (coverage_poly_seg2 is not None and not seg2_excluded) else 0.0
inside_area_seg4 = float(coverage_poly_seg4.area) if (coverage_poly_seg4 is not None and not seg4_excluded) else 0.0

# Sweep coverage (swarm as a whole, centroid along segment span)
sweep_seg1 = compute_sweep_coverage(SEG_IDX_1, workspace_polys[SEG_IDX_1], workspace_areas[SEG_IDX_1]) if not seg1_excluded else (None,0,0,0,float("nan"), None, None)
sweep_seg2 = compute_sweep_coverage(SEG_IDX_2, workspace_polys[SEG_IDX_2], workspace_areas[SEG_IDX_2]) if not seg2_excluded else (None,0,0,0,float("nan"), None, None)
sweep_seg4 = compute_sweep_coverage(SEG_IDX_4, workspace_polys[SEG_IDX_4], workspace_areas[SEG_IDX_4]) if not seg4_excluded else (None,0,0,0,float("nan"), None, None)

def _full_sweep_poly(sweep_tuple):
    """Full sweep = ribbon ∩ strip, already returned as last element."""
    if not sweep_tuple or len(sweep_tuple) < 7:
        return None
    return sweep_tuple[6]

sweep_full_seg1 = _full_sweep_poly(sweep_seg1)
sweep_full_seg2 = _full_sweep_poly(sweep_seg2)
sweep_full_seg4 = _full_sweep_poly(sweep_seg4)

# Overall coverage across the three segments combined (union of coverage + union of workspaces)
def _union_polys(polys):
    good = [p for p in polys if p is not None]
    if not good or unary_union is None:
        return None
    return unary_union(good)

included_workspaces = []
included_coverage = []
included_coverage_out = []
if not seg1_excluded:
    included_workspaces.append(workspace_polys[SEG_IDX_1])
    included_coverage.append(coverage_poly_seg1)
    included_coverage_out.append(coverage_out_seg1)
if not seg2_excluded:
    included_workspaces.append(workspace_polys[SEG_IDX_2])
    included_coverage.append(coverage_poly_seg2)
    included_coverage_out.append(coverage_out_seg2)
if not seg4_excluded:
    included_workspaces.append(workspace_polys[SEG_IDX_4])
    included_coverage.append(coverage_poly_seg4)
    included_coverage_out.append(coverage_out_seg4)

workspace_poly_overall = _union_polys(included_workspaces)
coverage_poly_overall  = _union_polys(included_coverage)
coverage_out_overall   = _union_polys(included_coverage_out)
if workspace_poly_overall is not None:
    workspace_area_overall = float(workspace_poly_overall.area)
else:
    # fallback to summed rectangle areas (rects don't overlap) for included segments
    workspace_area_overall = float(
        (workspace_areas[SEG_IDX_1] if not seg1_excluded else 0.0) +
        (workspace_areas[SEG_IDX_2] if not seg2_excluded else 0.0) +
        (workspace_areas[SEG_IDX_4] if not seg4_excluded else 0.0)
    )

if coverage_poly_overall is not None and workspace_area_overall > 0:
    # Inside overall = 按段 inside 面积求和
    covered_area_overall = inside_area_seg1 + inside_area_seg2 + inside_area_seg4
    # Outside overall = 按段 outside 面积求和
    covered_out_area_overall = float(
        (covered_out_area_seg1 if (coverage_out_seg1 is not None and not seg1_excluded) else 0.0) +
        (covered_out_area_seg2 if (coverage_out_seg2 is not None and not seg2_excluded) else 0.0) +
        (covered_out_area_seg4 if (coverage_out_seg4 is not None and not seg4_excluded) else 0.0)
    )
    effective_overall = max(covered_area_overall - covered_out_area_overall, 0.0) if SUBTRACT_OUTSIDE_COVERAGE else covered_area_overall
    coverage_pct_overall = 100.0 * effective_overall / workspace_area_overall
else:
    covered_area_overall = 0.0 if not math.isnan(workspace_area_overall) else float("nan")
    covered_out_area_overall = 0.0
    effective_overall = covered_area_overall
    coverage_pct_overall = float("nan")

# Overall sweep coverage (swarm as a whole) across included segments
sweep_inside_polys = []
sweep_out_polys = []
if not seg1_excluded:
    poly_in = _sweep_poly(sweep_seg1, 0)
    poly_out = _sweep_poly(sweep_seg1, 5)
    if poly_in is not None:
        sweep_inside_polys.append(poly_in)
    if poly_out is not None:
        sweep_out_polys.append(poly_out)
if not seg2_excluded:
    poly_in = _sweep_poly(sweep_seg2, 0)
    poly_out = _sweep_poly(sweep_seg2, 5)
    if poly_in is not None:
        sweep_inside_polys.append(poly_in)
    if poly_out is not None:
        sweep_out_polys.append(poly_out)
if not seg4_excluded:
    poly_in = _sweep_poly(sweep_seg4, 0)
    poly_out = _sweep_poly(sweep_seg4, 5)
    if poly_in is not None:
        sweep_inside_polys.append(poly_in)
    if poly_out is not None:
        sweep_out_polys.append(poly_out)

sweep_inside_union = _union_polys(sweep_inside_polys)
sweep_out_union = _union_polys(sweep_out_polys)
sweep_area_inside_overall = float(sweep_inside_union.area) if sweep_inside_union is not None else (
    _sweep_get(sweep_seg1, 3, 0.0) + _sweep_get(sweep_seg2, 3, 0.0) + _sweep_get(sweep_seg4, 3, 0.0)
)
sweep_area_outside_overall = float(sweep_out_union.area) if (sweep_out_union is not None and SUBTRACT_OUTSIDE_COVERAGE) else (
    _sweep_get(sweep_seg1, 2, 0.0) + _sweep_get(sweep_seg2, 2, 0.0) + _sweep_get(sweep_seg4, 2, 0.0)
)

sweep_effective_overall = max(sweep_area_inside_overall - sweep_area_outside_overall, 0.0) if SUBTRACT_OUTSIDE_COVERAGE else sweep_area_inside_overall
sweep_pct_overall = 100.0 * sweep_effective_overall / workspace_area_overall if workspace_area_overall > 0 else float("nan")

if exclude_set:
    exclude_label = ", ".join(f"#{idx+1}" for idx in sorted(exclude_set))
    exclude_phrase = f"excluding segments {exclude_label}"
    excluded_detail = f"excluded {excluded} of {total_in_run} samples nearest to {exclude_label}"
else:
    exclude_label = "none"
    exclude_phrase = "including all segments"
    excluded_detail = f"excluded {excluded} of {total_in_run} samples"

if VERBOSE:
    print("=== RUN METRICS (averages restricted to Run window) ===")
    if t0 is not None and t1 is not None:
        print(f"Run window (game time): start={t0:.3f}s, stop={t1:.3f}s, total={run_total_spent_time_s:.3f}s")
    else:
        print("No Run window found; using full extent for masks.")
    print(f"Average centroid→reference distance (Run, {exclude_phrase}): {avg_err_m:.3f} m")
    print(f"  ({excluded_detail})")
    print(f"Average inter-agent distance (main, Run): {avg_interagent_main_overall:.3f} m")
    print(f"Average inter-agent distance (swarm, Run): {avg_interagent_swarm_overall:.3f} m")
    print(f"Survived drones (present at stop & final g==1): {survivors} / {with_g} (with g-labels), total drones: {len(drone_tracks)}")

    print("\nCoverage windows:")
    print(f"  Segment 1 (idx {SEG_IDX_1}), length={workspace_lengths[SEG_IDX_1]:.3f} m, "
          f"width={SEG_WIDTHS[SEG_IDX_1]:.3f} m, area={workspace_areas[SEG_IDX_1]:.3f} m²")
    if seg1_excluded:
        print("    coverage: excluded")
    else:
        if SUBTRACT_OUTSIDE_COVERAGE:
            print(f"    coverage: {coverage_pct_seg1:.2f}% (inside {inside_area_seg1:.3f} m² "
                  f"minus outside {covered_out_area_seg1:.3f} m² -> effective {covered_area_seg1:.3f} m²)")
        else:
            print(f"    coverage: {coverage_pct_seg1:.2f}% (inside {inside_area_seg1:.3f} m²)")
        s1_eff = _fmt(_sweep_get(sweep_seg1, 3))
        s1_in  = _fmt(_sweep_get(sweep_seg1, 1))
        s1_out = _fmt(_sweep_get(sweep_seg1, 2))
        s1_pct = _fmt(_sweep_get(sweep_seg1, 4))
        print(f"    sweep (centroid-in-workspace): eff {s1_eff} m² inside {s1_in} m² "
              f"{'minus outside ' + s1_out + ' m²' if SUBTRACT_OUTSIDE_COVERAGE else ''} "
              f"({s1_pct}%)")

    print(f"  Segment 2 (idx {SEG_IDX_2}), length={workspace_lengths[SEG_IDX_2]:.3f} m, "
          f"width={SEG_WIDTHS[SEG_IDX_2]:.3f} m, area={workspace_areas[SEG_IDX_2]:.3f} m²")
    if seg2_excluded:
        print("    coverage: excluded")
    else:
        if SUBTRACT_OUTSIDE_COVERAGE:
            print(f"    coverage: {coverage_pct_seg2:.2f}% (inside {inside_area_seg2:.3f} m² "
                  f"minus outside {covered_out_area_seg2:.3f} m² -> effective {covered_area_seg2:.3f} m²)")
        else:
            print(f"    coverage: {coverage_pct_seg2:.2f}% (inside {inside_area_seg2:.3f} m²)")
        s2_eff = _fmt(_sweep_get(sweep_seg2, 3))
        s2_in  = _fmt(_sweep_get(sweep_seg2, 1))
        s2_out = _fmt(_sweep_get(sweep_seg2, 2))
        s2_pct = _fmt(_sweep_get(sweep_seg2, 4))
        print(f"    sweep (centroid-in-workspace): eff {s2_eff} m² inside {s2_in} m² "
              f"{'minus outside ' + s2_out + ' m²' if SUBTRACT_OUTSIDE_COVERAGE else ''} "
              f"({s2_pct}%)")

    print(f"  Segment 4 (idx {SEG_IDX_4}), length={workspace_lengths[SEG_IDX_4]:.3f} m, "
          f"width={SEG_WIDTHS[SEG_IDX_4]:.3f} m, area={workspace_areas[SEG_IDX_4]:.3f} m²")
    if seg4_excluded:
        print("    coverage: excluded")
    else:
        if SUBTRACT_OUTSIDE_COVERAGE:
            print(f"    coverage: {coverage_pct_seg4:.2f}% (inside {inside_area_seg4:.3f} m² "
                  f"minus outside {covered_out_area_seg4:.3f} m² -> effective {covered_area_seg4:.3f} m²)")
        else:
            print(f"    coverage: {coverage_pct_seg4:.2f}% (inside {inside_area_seg4:.3f} m²)")
        eff4 = _fmt(_sweep_get(sweep_seg4, 3))
        in4  = _fmt(_sweep_get(sweep_seg4, 1))
        out4 = _fmt(_sweep_get(sweep_seg4, 2))
        pct4 = _fmt(_sweep_get(sweep_seg4, 4))
        print(f"    sweep (centroid-in-workspace): eff {eff4} m² inside {in4} m² "
              f"{'minus outside ' + out4 + ' m²' if SUBTRACT_OUTSIDE_COVERAGE else ''} "
              f"({pct4}%)")

    print("\n  Overall (segments 1+2+4 union)")
    print(f"    workspace area={workspace_area_overall:.3f} m²")
    print(f"    coverage: {coverage_pct_overall:.2f}% (inside-sum {covered_area_overall:.3f} m²)")
    print(f"    sweep: {sweep_pct_overall:.2f}% (inside {sweep_area_inside_overall:.3f} m²"
          f"{' minus outside ' + format(sweep_area_outside_overall, '.3f') + ' m²' if SUBTRACT_OUTSIDE_COVERAGE else ''}, "
          f"effective {sweep_effective_overall:.3f} m²)")

    # Debug: print each drone’s last time and status
    print("\n--- Drone end-state summary ---")
    for name in sorted(drone_tracks.keys()):
        s = per_drone_status[name]
        print(f"{name:>12s}  t_last={s['t_last']:.6f}s   g_last={s['g_last']}   status={s['status']}")

    # Also report which track is embodied (if any)
    def embodied_track_names():
        return [n for n, d in drone_tracks.items() if is_embodied_track(n, d, embodied_id_meta, embodied_name_meta)]
    embodied_names = embodied_track_names()
    if embodied_names:
        print(f"\nEmbodied drone detected: {embodied_names[0]} (id={drone_tracks[embodied_names[0]].get('id')})")
    elif embodied_id_meta is not None or embodied_name_meta is not None:
        print(f"\nEmbodied metadata present but no exact match. meta id={embodied_id_meta}, name={embodied_name_meta}")

# Safe number formatting for metrics
def _num(val, nan_val=float("nan")):
    try:
        return float(val)
    except Exception:
        return nan_val
def _sweep_num(tup, idx):
    try:
        return float(tup[idx]) if (tup is not None and len(tup) > idx and tup[idx] is not None) else float("nan")
    except Exception:
        return float("nan")

# Sidecar metrics
metrics_txt = Path(OUT_ERR_PNG).with_suffix(".metrics.txt")
metrics_txt.write_text(
    f"file: {INPUT_JSON}\n"
    f"scene: {scene}\n"
    f"total_time_s: {_num(total_time_s):.3f}\n"
    f"avg_centroid_ref_dist_m: {_num(avg_err_m):.6f}\n"
    f"survivors_final_g1_present_at_stop: {survivors}\n"
    f"disconnected_at_end: {disconnected_end}\n"
    f"crashed_early: {crashed_early}\n"
    f"crashed_while_disconnected: {crashed_disconnected}\n"
    f"drones_with_g_label: {with_g}\n"
    f"total_drones: {len(drone_tracks)}\n"
    f"run_total_spent_time_s: {_num(run_total_spent_time_s):.6f}\n"
    f"split_metric: {_num(res.get('split_metric')):.6f}\n"
    f"split_metric_approach: {_num(res.get('split_metric_approach')):.6f}\n"
    f"avg_interagent_main_overall_m: {_num(avg_interagent_main_overall):.6f}\n"
    f"avg_interagent_swarm_overall_m: {_num(avg_interagent_swarm_overall):.6f}\n"
    f"seg1_length_m: {_num(workspace_lengths[SEG_IDX_1]):.6f}\n"
    f"seg1_workspace_width_m: {_num(SEG_WIDTHS[SEG_IDX_1]):.6f}\n"
    f"seg1_workspace_area_m2: {_num(workspace_areas[SEG_IDX_1]):.6f}\n"
    f"seg1_covered_area_m2: {_num(covered_area_seg1):.6f}\n"
    f"seg1_outside_area_m2: {_num(covered_out_area_seg1):.6f}\n"
    f"seg1_coverage_pct: {_num(coverage_pct_seg1):.6f}\n"
    f"seg1_sweep_inside_m2: {_sweep_num(sweep_seg1,1):.6f}\n"
    f"seg1_sweep_outside_m2: {_sweep_num(sweep_seg1,2):.6f}\n"
    f"seg1_sweep_effective_m2: {_sweep_num(sweep_seg1,3):.6f}\n"
    f"seg1_sweep_pct: {_sweep_num(sweep_seg1,4):.6f}\n"
    f"seg2_length_m: {_num(workspace_lengths[SEG_IDX_2]):.6f}\n"
    f"seg2_workspace_width_m: {_num(SEG_WIDTHS[SEG_IDX_2]):.6f}\n"
    f"seg2_workspace_area_m2: {_num(workspace_areas[SEG_IDX_2]):.6f}\n"
    f"seg2_covered_area_m2: {_num(covered_area_seg2):.6f}\n"
    f"seg2_outside_area_m2: {_num(covered_out_area_seg2):.6f}\n"
    f"seg2_coverage_pct: {_num(coverage_pct_seg2):.6f}\n"
    f"seg2_sweep_inside_m2: {_sweep_num(sweep_seg2,1):.6f}\n"
    f"seg2_sweep_outside_m2: {_sweep_num(sweep_seg2,2):.6f}\n"
    f"seg2_sweep_effective_m2: {_sweep_num(sweep_seg2,3):.6f}\n"
    f"seg2_sweep_pct: {_sweep_num(sweep_seg2,4):.6f}\n"
    f"seg4_length_m: {_num(workspace_lengths[SEG_IDX_4]):.6f}\n"
    f"seg4_workspace_width_m: {_num(SEG_WIDTHS[SEG_IDX_4]):.6f}\n"
    f"seg4_workspace_area_m2: {_num(workspace_areas[SEG_IDX_4]):.6f}\n"
    f"seg4_covered_area_m2: {_num(covered_area_seg4):.6f}\n"
    f"seg4_outside_area_m2: {_num(covered_out_area_seg4):.6f}\n"
    f"seg4_coverage_pct: {_num(coverage_pct_seg4):.6f}\n"
    f"seg4_sweep_inside_m2: {_sweep_num(sweep_seg4,1):.6f}\n"
    f"seg4_sweep_outside_m2: {_sweep_num(sweep_seg4,2):.6f}\n"
    f"seg4_sweep_effective_m2: {_sweep_num(sweep_seg4,3):.6f}\n"
    f"seg4_sweep_pct: {_sweep_num(sweep_seg4,4):.6f}\n"
    f"overall_workspace_area_m2: {_num(workspace_area_overall):.6f}\n"
    f"overall_covered_area_m2: {_num(covered_area_overall):.6f}\n"
    f"overall_outside_area_m2: {_num(covered_out_area_overall):.6f}\n"
    f"overall_coverage_pct: {_num(coverage_pct_overall):.6f}\n"
    f"overall_sweep_inside_m2: {_num(sweep_area_inside_overall):.6f}\n"
    f"overall_sweep_outside_m2: {_num(sweep_area_outside_overall):.6f}\n"
    f"overall_sweep_effective_m2: {_num(sweep_effective_overall):.6f}\n"
    f"overall_sweep_pct: {_num(sweep_pct_overall):.6f}\n"
)

key_metrics = {
    "file": str(INPUT_JSON),
    "scene": scene,
    "avg_centroid_ref_dist_m": _num(avg_err_m),
    "run_total_spent_time_s": _num(run_total_spent_time_s),
    "survivors": _num(survivors),
    "disconnected": _num(disconnected_end),
    "crashed_early": _num(crashed_early),
    "crashed_disconnected": _num(crashed_disconnected),
    "crashed_total": _num(crashed_total),
    "total_drones": _num(len(drone_tracks)),
    "split_metric": _num(res.get("split_metric")),
    "split_metric_approach": _num(res.get("split_metric_approach")),
    "overall_sweep_pct": _num(sweep_pct_overall),
}

if METRICS_ONLY:
    print(json.dumps(key_metrics, indent=2))
    sys.exit(0)

# -------- Plot helpers --------
def game_time_to_axis_x_val(t_game):
    return game_time_to_axis_x(t_game, use_time, sample_hz)

x0_line = game_time_to_axis_x_val(t0)
x1_line = game_time_to_axis_x_val(t1)

# Build contiguous spans of the same nearest-segment index for banding on the x-axis.
def _segment_spans(times_arr, seg_idx_arr):
    """
    Return [(x_start, x_end, seg_idx), ...] covering the timeline with
    the nearest reference segment for the centroid.
    """
    if len(times_arr) == 0 or len(seg_idx_arr) == 0:
        return []
    times_arr = np.asarray(times_arr, dtype=float)
    seg_idx_arr = np.asarray(seg_idx_arr, dtype=int)
    if times_arr.shape[0] != seg_idx_arr.shape[0]:
        return []
    # Build edges halfway between samples; extend the last edge by median dt (or 1.0 fallback)
    edges = np.empty(len(times_arr) + 1, dtype=float)
    if len(times_arr) > 1:
        midpoints = 0.5 * (times_arr[:-1] + times_arr[1:])
        edges[1:-1] = midpoints
        dt = np.median(np.diff(times_arr))
        edges[-1] = times_arr[-1] + (dt if dt > 0 else 1.0)
    else:
        edges[1:-1] = times_arr[0]
        edges[-1] = times_arr[0] + 1.0
    edges[0] = times_arr[0]

    spans = []
    start_idx = 0
    for i in range(1, len(seg_idx_arr)):
        if seg_idx_arr[i] != seg_idx_arr[start_idx]:
            spans.append((edges[start_idx], edges[i], int(seg_idx_arr[start_idx])))
            start_idx = i
    spans.append((edges[start_idx], edges[-1], int(seg_idx_arr[start_idx])))
    return spans

# ----- Color mapping: unique color per drone (kept consistent), embodied emphasized -----
all_names_sorted = sorted(drone_tracks.keys())

# Use a qualitative colormap and sample it cyclically
cmap = plt.get_cmap('tab20')  # falls back gracefully if tab20 exists; widely available
N = getattr(cmap, 'N', 20)    # number of distinct colors in the map (tab20 -> 20)

name_to_color = {name: cmap(i % N) for i, name in enumerate(all_names_sorted)}
embodied_timeline = embodied_name_timeline(times, drone_tracks, use_time, sample_hz)

# 1) Trajectories (X-Z)
saved_files = []
made_any_fig = False

if FIG_FLAGS.get("traj", True):
    plt.figure(figsize=(8, 8))
    for name in all_names_sorted:
        d = drone_tracks[name]
        is_emb = is_embodied_track(name, d, embodied_id_meta, embodied_name_meta)
        c = name_to_color[name]

        # Path styling
        plt.plot(d["x"], d["z"], linewidth=1.8, alpha=0.28, color=c, zorder=1, label=None)
        # Overlay time-varying embodied segments, if present
        plot_embodied_segments(plt.gca(), d, color=c, linewidth=3.0, alpha=0.95)

        # Endpoint marker by status (use the same per-drone color)
        if len(d["x"]) > 0:
            end_x, end_z = d["x"][-1], d["z"][-1]
            st = per_drone_status[name]['status']
            z = 4 if is_emb else 2
            size_boost = 1.3 if is_emb else 1.0
            if st == 'survivor':
                plt.scatter(end_x, end_z, s=36*size_boost, marker="o", color=c, edgecolors='k', linewidths=0.3, zorder=z)
            elif st == 'disconnected_at_end':
                plt.scatter(end_x, end_z, s=50*size_boost, marker="^", color=c, edgecolors='k', linewidths=0.3, zorder=z)
            elif st == 'crashed_or_vanished_early':
                plt.scatter(end_x, end_z, s=60*size_boost, marker="X", color=c, edgecolors='k', linewidths=0.3, zorder=z)
            else:  # vanished_while_disconnected
                plt.scatter(end_x, end_z, s=36*size_boost, marker="s", color=c, edgecolors='k', linewidths=0.3, zorder=z)

    plt.plot(ref_poly[:,0], ref_poly[:,1], linewidth=3, linestyle="--", label=f"Reference (×{REF_SCALE})")
    plt.scatter(ref_poly[:,0], ref_poly[:,1], s=20)

    for idx, seg_idx in enumerate([SEG_IDX_1, SEG_IDX_2, SEG_IDX_4]):
        coords = rects_coords[seg_idx]
        lbl = "Workspace segments 1/2/4" if idx == 0 else None
        plt.plot(coords[:,0], coords[:,1],
                 linewidth=1.5, linestyle="-", color="k", label=lbl)

    plt.plot(centroid_x, centroid_z, linewidth=3, label="Swarm centroid (main group)")
    plt.scatter([centroid_x[0]],[centroid_z[0]], s=50, marker="o", label="Centroid start")
    plt.scatter([centroid_x[-1]],[centroid_z[-1]], s=50, marker="x", label="Centroid end")

    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("X (m)"); plt.ylabel("Z (m)")
    plt.title(f"Trajectories & Centroid vs Reference — {scene}\nFile: {INPUT_JSON.name}")
    plt.grid(True, alpha=0.3)

    legend_path = [
        Line2D([0],[0], color='k', lw=1.8, alpha=0.7, label='Other drones (various colors)'),
        Line2D([0],[0], color='k', lw=3.0, label='Embodied segments (e==1)'),
    ]
    legend_endpoints = [
        Line2D([0],[0], marker='o', linestyle='None', label='Endpoint: survivor'),
        Line2D([0],[0], marker='^', linestyle='None', label='Endpoint: disconnected@stop'),
        Line2D([0],[0], marker='X', linestyle='None', label='Endpoint: crashed/vanished early'),
        Line2D([0],[0], marker='s', linestyle='None', label='Endpoint: vanished while disconnected'),
    ]
    h0, l0 = plt.gca().get_legend_handles_labels()
    plt.legend(handles=legend_path + legend_endpoints + h0, loc="best")

    a, b, ab, L, L2, t_hat, n_hat, M = _segment_endpoints(ref_poly, 10)
    xmin, xmax = np.min(ref_poly[:,0]), np.max(ref_poly[:,0])
    zmin, zmax = np.min(ref_poly[:,1]), np.max(ref_poly[:,1])
    disp_len = 0.6 * max(xmax - xmin, zmax - zmin)
    p1 = M - n_hat * disp_len
    p2 = M + n_hat * disp_len
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], linestyle=":", linewidth=2.0, color="k", label="Midpoint line (seg 10)")

    plt.tight_layout(); plt.savefig(OUT_TRAJ_PNG, dpi=150)
    saved_files.append(OUT_TRAJ_PNG)
    made_any_fig = True

# 2) Centroid cross-track error vs time (with Run shading)
if FIG_FLAGS.get("cte", True):
    fig_cte, ax_cte = plt.subplots(figsize=(9, 5))
    ax_cte.plot(times, centroid_err, label="Centroid CTE")
    if x0_line is not None and x1_line is not None and x1_line >= x0_line:
        ax_cte.axvspan(x0_line, x1_line, alpha=0.15, label=f"Run window ({run_total_spent_time_s:.2f}s)")
        ax_cte.axvline(x0_line, linestyle="--"); ax_cte.axvline(x1_line, linestyle="--")

    # Mark which reference segment the centroid is closest to along the x-axis with a thin color band.
    seg_spans = _segment_spans(times, centroid_segidx)
    seg_handles = []
    if seg_spans:
        seg_cmap = plt.get_cmap('tab10')
        uniq_segs = sorted({seg for _, _, seg in seg_spans})
        for start, end, seg in seg_spans:
            col = seg_cmap(seg % seg_cmap.N)
            ax_cte.axvspan(start, end, ymin=0.0, ymax=0.045, color=col, alpha=0.18, linewidth=0)
        seg_handles = [
            Line2D([0],[0], color=seg_cmap(seg % seg_cmap.N), lw=6, label=f"Nearest ref segment #{seg+1}")
            for seg in uniq_segs
        ]

    ax_cte.set_xlabel("Time (s)" if use_time else f"Frame index (~{sample_hz:.1f} Hz)")
    ax_cte.set_ylabel("Centroid cross-track error (m)")
    ax_cte.set_title("Centroid cross-track error vs time (main group only)")
    ax_cte.grid(True, alpha=0.3)
    h_cte, l_cte = ax_cte.get_legend_handles_labels()
    ax_cte.legend(
        handles=h_cte + seg_handles,
        labels=l_cte + [h.get_label() for h in seg_handles],
        loc="best"
    )
    fig_cte.tight_layout(); fig_cte.savefig(OUT_ERR_PNG, dpi=150)
    saved_files.append(OUT_ERR_PNG)
    made_any_fig = True

# 3) Avg inter-agent distance (main vs swarm) with Run shading
if FIG_FLAGS.get("interdist", True):
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
    saved_files.append(OUT_INTERDIST_BOTH_PNG)
    made_any_fig = True

# 4) Swarm width (main group) relative to embodied orientation
if FIG_FLAGS.get("width_emb", True):
    plt.figure(figsize=(9, 5))
    plt.plot(swarm_width_times_emb, swarm_width_vals_emb, label="Swarm width (perpendicular span)", linewidth=2.2)
    if x0_line is not None and x1_line is not None and x1_line >= x0_line:
        plt.axvspan(x0_line, x1_line, alpha=0.15, label=f"Run window ({run_total_spent_time_s:.2f}s)")
        plt.axvline(x0_line, linestyle="--"); plt.axvline(x1_line, linestyle="--")
    plt.xlabel("Time (s)" if use_time else f"Frame index (~{sample_hz:.1f} Hz)")
    plt.ylabel("Width (m)")
    plt.title("Swarm width vs time (main group, perpendicular to embodied forward)")
    plt.grid(True, alpha=0.3); plt.legend(loc="best")
    plt.tight_layout(); plt.savefig(OUT_WIDTH_EMB_PNG, dpi=150)
    saved_files.append(OUT_WIDTH_EMB_PNG)
    made_any_fig = True

# 5) Swarm width (main group) perpendicular to nearest reference segment
if FIG_FLAGS.get("width_seg", True):
    plt.figure(figsize=(9, 5))
    plt.plot(swarm_width_times_seg, swarm_width_vals_seg, label="Swarm width ⟂ nearest segment", linewidth=2.2)
    if x0_line is not None and x1_line is not None and x1_line >= x0_line:
        plt.axvspan(x0_line, x1_line, alpha=0.15, label=f"Run window ({run_total_spent_time_s:.2f}s)")
        plt.axvline(x0_line, linestyle="--"); plt.axvline(x1_line, linestyle="--")
    plt.xlabel("Time (s)" if use_time else f"Frame index (~{sample_hz:.1f} Hz)")
    plt.ylabel("Width (m)")
    plt.title("Swarm width vs time (main group, ⟂ nearest segment)")
    plt.grid(True, alpha=0.3); plt.legend(loc="best")
    plt.tight_layout(); plt.savefig(OUT_WIDTH_SEG_PNG, dpi=150)
    saved_files.append(OUT_WIDTH_SEG_PNG)
    made_any_fig = True

if saved_files:
    print("Saved:", ", ".join(saved_files))
elif not FIG_FLAGS.get("interactive", True):
    print("No static figures generated (interactive only).")

# If we only generated static figures (no interactive), show them now.
if made_any_fig and not FIG_FLAGS.get("interactive", True):
    _show_with_raise()

# =========================
# Interactive time slider: reveal trajectories up to time T
# =========================
if FIG_FLAGS.get("interactive", True):
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider, Button

    # --- Which drone is "embodied"? pull from JSON if recorded; else fallback by name ---
    embodied_names_static = set()
    if "trajectories" in data:
        for i, traj in enumerate(data["trajectories"]):
            if traj.get("isEmbodied") or traj.get("embodied") or traj.get("is_embodied"):
                embodied_names_static.add(traj.get("name", f"id:{traj.get('id', i)}"))
    # Fallback heuristic
    for n in list(drone_tracks.keys()):
        lo = n.lower()
        if "embodied" in lo or "player" in lo or "ego" in lo:
            embodied_names_static.add(n)

    def _is_embodied_at_time(track, T, sample_hz):
        e = track.get("e", None)
        if e is None or len(e) == 0:
            return bool(track.get("embodied", False))
        ts = _time_array_for(track)
        if len(ts) == 0:
            return False
        idx = np.searchsorted(ts, T, side='right') - 1
        if idx < 0:
            return False
        if idx >= len(e):
            idx = len(e) - 1
        return bool(np.asarray(e, dtype=float)[idx] >= 0.5)

    def _time_array_for(dr):
        """Return per-sample time array for a drone track."""
        if dr.get("t") is not None and len(dr["t"]) > 0:
            return np.asarray(dr["t"], dtype=float)
        n = len(dr.get("x", []))
        return np.arange(n, dtype=float) / float(sample_hz)

    # Build a unified min/max time range (prefer Run window if present)
    per_drone_times = {name: _time_array_for(d) for name, d in drone_tracks.items() if len(d["x"]) > 0}
    if not per_drone_times:
        raise RuntimeError("No drone samples to visualize in slider view.")

    global_t_min = min(ts[0] for ts in per_drone_times.values() if len(ts) > 0)
    global_t_max = max(ts[-1] for ts in per_drone_times.values() if len(ts) > 0)

    # If Run window exists, limit slider to that; else use full extent
    slider_t0 = x0_line if (x0_line is not None) else global_t_min
    slider_t1 = x1_line if (x1_line is not None) else global_t_max
    if slider_t1 < slider_t0:  # just in case
        slider_t0, slider_t1 = global_t_min, global_t_max

    # ---- Figure & axes layout (leave room for the slider and button) ----
    fig, ax = plt.subplots(figsize=(8.8, 8.8))
    plt.subplots_adjust(left=0.08, right=0.98, top=0.93, bottom=0.18)

    # Colormap for per-drone colors (keeps colors distinct)
    try:
        cmap = plt.get_cmap('tab20')
        names_sorted = sorted(drone_tracks.keys())
        N = getattr(cmap, 'N', 20)
        name_to_color = {n: cmap(i % N) for i, n in enumerate(names_sorted)}
    except Exception:
        name_to_color = {}

    # Reference path
    ax.plot(ref_poly[:,0], ref_poly[:,1], linestyle="--", linewidth=2, label=f"Reference (×{REF_SCALE})")
    ax.scatter(ref_poly[:,0], ref_poly[:,1], s=14, alpha=0.7)

    # workspace rectangles for segments 1, 2, 4 (same marking style as static plot)
    for idx, seg_idx in enumerate([SEG_IDX_1, SEG_IDX_2, SEG_IDX_4]):
        coords = rects_coords[seg_idx]
        lbl = "Workspace segments 1/2/4" if idx == 0 else None
        ax.plot(coords[:,0], coords[:,1],
                linewidth=1.5, linestyle="-", color="k", label=lbl)

    # Coverage patches (initially hidden)
    coverage_patches = []
    # First: per-segment coverage in one color, then overall union in another
    coverage_polys_for_plot = [
        (coverage_poly_seg1, dict(facecolor='C1', edgecolor='none', alpha=0.25, zorder=1)),
        (coverage_poly_seg2, dict(facecolor='C1', edgecolor='none', alpha=0.25, zorder=1)),
        (coverage_poly_seg4, dict(facecolor='C1', edgecolor='none', alpha=0.25, zorder=1)),
        (coverage_out_seg1, dict(facecolor='#ff6666', edgecolor='none', alpha=0.35, zorder=0, label='Outside coverage')),
        (coverage_out_seg2, dict(facecolor='#ff6666', edgecolor='none', alpha=0.35, zorder=0, label='Outside coverage')),
        (coverage_out_seg4, dict(facecolor='#ff6666', edgecolor='none', alpha=0.35, zorder=0, label='Outside coverage')),
        (coverage_poly_overall, dict(facecolor='C2', edgecolor='none', alpha=0.20, zorder=1)),
    ]
    coverage_sweep_patches = []
    # Visualize sweep inside / outside separately per segment
    coverage_sweep_polys_for_plot = [
        # Segment 1
        (_sweep_poly(sweep_seg1, 0), dict(facecolor='#7fc97f', edgecolor='none', alpha=0.35, zorder=0, label='Sweep inside seg1')),
        (_sweep_poly(sweep_seg1, 5), dict(facecolor='#ff9999', edgecolor='none', alpha=0.35, zorder=0, label='Sweep outside seg1')),
        # Segment 2
        (_sweep_poly(sweep_seg2, 0), dict(facecolor='#7fc97f', edgecolor='none', alpha=0.35, zorder=0, label='Sweep inside seg2')),
        (_sweep_poly(sweep_seg2, 5), dict(facecolor='#ff9999', edgecolor='none', alpha=0.35, zorder=0, label='Sweep outside seg2')),
        # Segment 4
        (_sweep_poly(sweep_seg4, 0), dict(facecolor='#7fc97f', edgecolor='none', alpha=0.35, zorder=0, label='Sweep inside seg4')),
        (_sweep_poly(sweep_seg4, 5), dict(facecolor='#ff9999', edgecolor='none', alpha=0.35, zorder=0, label='Sweep outside seg4')),
    ]
    for poly, opts in coverage_polys_for_plot:
        if poly is None or Point is None:
            continue
        patches_here = shapely_to_patches(
            poly, **opts
        )
        for p in patches_here:
            p.set_visible(False)
            ax.add_patch(p)
            coverage_patches.append(p)

    for poly, opts in coverage_sweep_polys_for_plot:
        if poly is None or Point is None:
            continue
        patches_here = shapely_to_patches(poly, **opts)
        for p in patches_here:
            p.set_visible(False)
            ax.add_patch(p)
            coverage_sweep_patches.append(p)

    # Prepare a colored line for each drone and a per-drone endpoint marker
    drone_lines = {}
    drone_markers = {}
    line_widths = {}
    line_sizes = {}
    line_alphas = {}
    for name, d in drone_tracks.items():
        col = name_to_color.get(name, None)
        has_any_emb = (d.get("e") is not None and np.any(np.asarray(d.get("e")) == 1)) or (name in embodied_names_static)
        lw_base = 1.8
        lw_emb = 3.2
        size_base = 28
        size_emb = 48
        alpha_base = 0.32
        alpha_emb = 0.95
        line, = ax.plot([], [], linewidth=lw_base, alpha=alpha_base, label=name, color=col)
        mark = ax.scatter([], [], s=size_base, marker='o', color=col if col else None, zorder=3)
        drone_lines[name] = line
        drone_markers[name] = mark
        line_widths[name] = (lw_base, lw_emb)
        line_sizes[name] = (size_base, size_emb)
        line_alphas[name] = (alpha_base, alpha_emb)

    # Centroid line up to T + markers
    centroid_line, = ax.plot([], [], linewidth=3.0, color='k', label="Centroid (to T)")
    centroid_start = ax.scatter([], [], s=40, marker='o', color='k', zorder=4, label="Centroid start")
    centroid_end   = ax.scatter([], [], s=56, marker='x', color='k', zorder=4, label="Centroid @T")

    # Formatting
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title(f"Interactive Trajectories — scrub time with slider\n{scene}  |  File: {INPUT_JSON.name}")
    ax.grid(True, alpha=0.3)

    # Slider axis
    ax_T = plt.axes([0.08, 0.08, 0.70, 0.05])  # [left, bottom, width, height]
    slider = Slider(ax=ax_T, label="Time (s)", valmin=slider_t0, valmax=slider_t1, valinit=slider_t0)

    def _clip_to_time(xs, zs, ts, T):
        """Return x,z arrays truncated to last index with t <= T (inclusive)."""
        if ts is None or len(ts) == 0:
            return np.array([]), np.array([])
        idx = np.searchsorted(ts, T, side='right') - 1
        if idx < 0:
            return np.array([]), np.array([])
        return xs[:idx+1], zs[:idx+1]

    def _update_plot(T):
        # Update each drone
        for name, d in drone_tracks.items():
            xs = np.asarray(d["x"], dtype=float)
            zs = np.asarray(d["z"], dtype=float)
            ts = per_drone_times[name]

            xseg, zseg = _clip_to_time(xs, zs, ts, T)
            drone_lines[name].set_data(xseg, zseg)

            # Move endpoint marker to last visible point
            if len(xseg) > 0:
                drone_markers[name].set_offsets(np.c_[xseg[-1], zseg[-1]])
            else:
                drone_markers[name].set_offsets(np.c_[[], []])  # hide

            # Update styling based on whether this drone is embodied at time T
            is_emb_now = _is_embodied_at_time(d, T, sample_hz)
            lw_base, lw_emb = line_widths[name]
            size_base, size_emb = line_sizes[name]
            alpha_base, alpha_emb = line_alphas[name]
            drone_lines[name].set_linewidth(lw_emb if is_emb_now else lw_base)
            drone_markers[name].set_sizes([size_emb if is_emb_now else size_base])
            drone_lines[name].set_alpha(alpha_emb if is_emb_now else alpha_base)

        # Update centroid up to T
        if len(times) > 0:
            idx_c = np.searchsorted(times, T, side='right') - 1
            if idx_c >= 0:
                cx = centroid_x[:idx_c+1]
                cz = centroid_z[:idx_c+1]
                centroid_line.set_data(cx, cz)
                centroid_start.set_offsets(np.c_[centroid_x[0], centroid_z[0]])
                centroid_end.set_offsets(np.c_[centroid_x[idx_c], centroid_z[idx_c]])
            else:
                centroid_line.set_data([], [])
                centroid_start.set_offsets(np.c_[[], []])
                centroid_end.set_offsets(np.c_[[], []])

        fig.canvas.draw_idle()

    # Initialize plot at slider start
    _update_plot(slider.val)

    # Connect slider
    def on_slider(val):
        _update_plot(val)
    slider.on_changed(on_slider)

    # ----- Button to toggle coverage area -----
    ax_btn = plt.axes([0.80, 0.08, 0.12, 0.05])  # [left, bottom, width, height]
    btn = Button(ax_btn, 'Toggle coverage')

    coverage_visible = [False]
    coverage_sweep_visible = [False]

    def on_btn_clicked(event):
        coverage_visible[0] = not coverage_visible[0]
        for p in coverage_patches:
            p.set_visible(coverage_visible[0])
        for p in coverage_sweep_patches:
            p.set_visible(False)  # hide sweep when coverage toggled on
        fig.canvas.draw_idle()

    btn.on_clicked(on_btn_clicked)

    # ----- Button to toggle sweep coverage area -----
    ax_btn_sweep = plt.axes([0.80, 0.02, 0.12, 0.05])
    btn_sweep = Button(ax_btn_sweep, 'Toggle sweep')

    def on_btn_sweep(event):
        coverage_sweep_visible[0] = not coverage_sweep_visible[0]
        for p in coverage_sweep_patches:
            p.set_visible(coverage_sweep_visible[0])
        for p in coverage_patches:
            p.set_visible(False)  # hide standard coverage when sweep shown
        fig.canvas.draw_idle()

    btn_sweep.on_clicked(on_btn_sweep)

    # Legend (comment out if too many drones)
    ax.legend(loc="best", fontsize=8, ncol=1)

    _show_with_raise()
