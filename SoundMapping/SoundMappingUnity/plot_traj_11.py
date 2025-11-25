# === Averages limited to the Run window (start..stop) + robust end-state classification ===
import json, math, glob, os
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon as MplPolygon
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

# Segment indices (0-based) in REF_STEPS
SEG_IDX_1 = 0   # first line of REF_STEPS
SEG_IDX_2 = 1   # second line
SEG_IDX_4 = 3   # fourth line

# Per-segment workspace widths in *world meters*, scaled by REF_SCALE to match ref_poly units
SEG_WIDTHS = {
    SEG_IDX_1: 14.0 * REF_SCALE,  # originally 16 m wide
    SEG_IDX_2: 18.0 * REF_SCALE,  # 18 m wide
    SEG_IDX_4: 20.0 * REF_SCALE,  # 20 m wide
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
          "will be skipped. Install via `pip install shapely` to enable it.")


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

# coverage test no haptic
# candidates = sorted(glob.glob("..."), key=os.path.getmtime, reverse=True)

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
        drone_tracks[name] = {
            "id": traj.get("id", None),
            "embodied": bool(traj.get("embodied", False)),
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
            "id": entry.get("droneId", None),
            "embodied": bool(entry.get("embodied", False)),
            "t": t_here,
            "x": np.array(x_arr, dtype=float),
            "z": np.array(z_arr, dtype=float),
            "g": np.array(g_arr, dtype=float) if any(v is not None for v in g_arr) else None,
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

# -------- Reference path --------
pts = [(0.0, 60.0)]
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

# Average centroid→reference distance (run mask) EXCLUDING the 3rd segment (index 2)
EXCLUDE_SEG_INDEX = 2
if np.any(mask_cte):
    mask_not_third = (centroid_segidx != EXCLUDE_SEG_INDEX)
    mask_for_avg = mask_cte & mask_not_third
    avg_err_m = float(np.mean(centroid_err[mask_for_avg])) if np.any(mask_for_avg) else float("nan")
    excluded = int(np.sum(mask_cte & ~mask_not_third))
    total_in_run = int(np.sum(mask_cte))
else:
    avg_err_m = float("nan")
    excluded = 0
    total_in_run = 0

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
    Returns:
      (union_poly, covered_area, coverage_pct)
    """
    if Polygon is None or Point is None or unary_union is None:
        return None, 0.0, float("nan")

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
        return None, 0.0, 0.0

    disks = []
    for (x_p, z_p) in positions:
        disk = Point(x_p, z_p).buffer(SENSING_RADIUS, resolution=32)
        if workspace_poly is not None:
            disk = disk.intersection(workspace_poly)
        if not disk.is_empty:
            disks.append(disk)

    if not disks:
        return None, 0.0, 0.0

    union_poly = unary_union(disks)
    covered_area = float(union_poly.area)
    coverage_pct = 100.0 * covered_area / float(workspace_area) if workspace_area > 0 else float("nan")
    return union_poly, covered_area, coverage_pct

# Compute coverage for three segments: 1, 2, and 4
coverage_poly_seg1, covered_area_seg1, coverage_pct_seg1 = compute_segment_coverage(
    SEG_IDX_1, workspace_polys[SEG_IDX_1], workspace_areas[SEG_IDX_1]
)
coverage_poly_seg2, covered_area_seg2, coverage_pct_seg2 = compute_segment_coverage(
    SEG_IDX_2, workspace_polys[SEG_IDX_2], workspace_areas[SEG_IDX_2]
)
coverage_poly_seg4, covered_area_seg4, coverage_pct_seg4 = compute_segment_coverage(
    SEG_IDX_4, workspace_polys[SEG_IDX_4], workspace_areas[SEG_IDX_4]
)

print("=== RUN METRICS (averages restricted to Run window) ===")
if t0 is not None and t1 is not None:
    print(f"Run window (game time): start={t0:.3f}s, stop={t1:.3f}s, total={run_total_spent_time_s:.3f}s")
else:
    print("No Run window found; using full extent for masks.")
print(f"Average centroid→reference distance (Run, excluding segment #3): {avg_err_m:.3f} m")
print(f"  (excluded {excluded} of {total_in_run} samples whose nearest segment was the 3rd)")
print(f"Average inter-agent distance (main, Run): {avg_interagent_main_overall:.3f} m")
print(f"Average inter-agent distance (swarm, Run): {avg_interagent_swarm_overall:.3f} m")
print(f"Survived drones (present at stop & final g==1): {survivors} / {with_g} (with g-labels), total drones: {len(drone_tracks)}")

print("\nCoverage windows:")
print(f"  Segment 1 (idx {SEG_IDX_1}), length={workspace_lengths[SEG_IDX_1]:.3f} m, "
      f"width={SEG_WIDTHS[SEG_IDX_1]:.3f} m, area={workspace_areas[SEG_IDX_1]:.3f} m²")
print(f"    coverage: {coverage_pct_seg1:.2f}% (covered {covered_area_seg1:.3f} m²)")

print(f"  Segment 2 (idx {SEG_IDX_2}), length={workspace_lengths[SEG_IDX_2]:.3f} m, "
      f"width={SEG_WIDTHS[SEG_IDX_2]:.3f} m, area={workspace_areas[SEG_IDX_2]:.3f} m²")
print(f"    coverage: {coverage_pct_seg2:.2f}% (covered {covered_area_seg2:.3f} m²)")

print(f"  Segment 4 (idx {SEG_IDX_4}), length={workspace_lengths[SEG_IDX_4]:.3f} m, "
      f"width={SEG_WIDTHS[SEG_IDX_4]:.3f} m, area={workspace_areas[SEG_IDX_4]:.3f} m²")
print(f"    coverage: {coverage_pct_seg4:.2f}% (covered {covered_area_seg4:.3f} m²)")

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
    f"seg1_length_m: {workspace_lengths[SEG_IDX_1]:.6f}\n"
    f"seg1_workspace_width_m: {SEG_WIDTHS[SEG_IDX_1]:.6f}\n"
    f"seg1_workspace_area_m2: {workspace_areas[SEG_IDX_1]:.6f}\n"
    f"seg1_covered_area_m2: {covered_area_seg1:.6f}\n"
    f"seg1_coverage_pct: {coverage_pct_seg1:.6f}\n"
    f"seg2_length_m: {workspace_lengths[SEG_IDX_2]:.6f}\n"
    f"seg2_workspace_width_m: {SEG_WIDTHS[SEG_IDX_2]:.6f}\n"
    f"seg2_workspace_area_m2: {workspace_areas[SEG_IDX_2]:.6f}\n"
    f"seg2_covered_area_m2: {covered_area_seg2:.6f}\n"
    f"seg2_coverage_pct: {coverage_pct_seg2:.6f}\n"
    f"seg4_length_m: {workspace_lengths[SEG_IDX_4]:.6f}\n"
    f"seg4_workspace_width_m: {SEG_WIDTHS[SEG_IDX_4]:.6f}\n"
    f"seg4_workspace_area_m2: {workspace_areas[SEG_IDX_4]:.6f}\n"
    f"seg4_covered_area_m2: {covered_area_seg4:.6f}\n"
    f"seg4_coverage_pct: {coverage_pct_seg4:.6f}\n"
)

# -------- Plot helpers --------
def game_time_to_axis_x_val(t_game):
    return game_time_to_axis_x(t_game, use_time, sample_hz)

x0_line = game_time_to_axis_x_val(t0)
x1_line = game_time_to_axis_x_val(t1)

# ----- Color mapping: unique color per drone (kept consistent), embodied emphasized -----
all_names_sorted = sorted(drone_tracks.keys())

# Use a qualitative colormap and sample it cyclically
cmap = plt.get_cmap('tab20')  # falls back gracefully if tab20 exists; widely available
N = getattr(cmap, 'N', 20)    # number of distinct colors in the map (tab20 -> 20)

name_to_color = {name: cmap(i % N) for i, name in enumerate(all_names_sorted)}

# 1) Trajectories (X-Z)
plt.figure(figsize=(8, 8))
for name in all_names_sorted:
    d = drone_tracks[name]
    is_emb = is_embodied_track(name, d, embodied_id_meta, embodied_name_meta)
    c = name_to_color[name]

    # Path styling
    if is_emb:
        # Emphasize embodied by thicker line & stronger alpha but keep its own color
        plt.plot(d["x"], d["z"], linewidth=3.0, alpha=0.95, color=c, zorder=3, label=None)
    else:
        plt.plot(d["x"], d["z"], linewidth=1.8, alpha=0.70, color=c, zorder=1, label=None)

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

# reference polyline
plt.plot(ref_poly[:,0], ref_poly[:,1], linewidth=3, linestyle="--", label=f"Reference (×{REF_SCALE})")
plt.scatter(ref_poly[:,0], ref_poly[:,1], s=20)

# workspace rectangles for segments 1, 2, 4 (marked same way as before, but for all three)
for idx, seg_idx in enumerate([SEG_IDX_1, SEG_IDX_2, SEG_IDX_4]):
    coords = rects_coords[seg_idx]
    lbl = "Workspace segments 1/2/4" if idx == 0 else None
    plt.plot(coords[:,0], coords[:,1],
             linewidth=1.5, linestyle="-", color="k", label=lbl)

# centroid path + start/end
plt.plot(centroid_x, centroid_z, linewidth=3, label="Swarm centroid (main group)")
plt.scatter([centroid_x[0]],[centroid_z[0]], s=50, marker="o", label="Centroid start")
plt.scatter([centroid_x[-1]],[centroid_z[-1]], s=50, marker="x", label="Centroid end")

plt.gca().set_aspect("equal", adjustable="box")
plt.xlabel("X (m)"); plt.ylabel("Z (m)")
plt.title(f"Trajectories & Centroid vs Reference — {scene}\nFile: {INPUT_JSON.name}")
plt.grid(True, alpha=0.3)

# Legend entries: endpoint semantics + path styling
legend_path = [
    Line2D([0],[0], color='k', lw=1.8, alpha=0.7, label='Other drones (various colors)'),
    Line2D([0],[0], color='k', lw=3.0, label='Embodied drone path'),
]
legend_endpoints = [
    Line2D([0],[0], marker='o', linestyle='None', label='Endpoint: survivor'),
    Line2D([0],[0], marker='^', linestyle='None', label='Endpoint: disconnected@stop'),
    Line2D([0],[0], marker='X', linestyle='None', label='Endpoint: crashed/vanished early'),
    Line2D([0],[0], marker='s', linestyle='None', label='Endpoint: vanished while disconnected'),
]
h0, l0 = plt.gca().get_legend_handles_labels()
plt.legend(handles=legend_path + legend_endpoints + h0, loc="best")

# --- draw midpoint line of segment 10 for visualization ---
a, b, ab, L, L2, t_hat, n_hat, M = _segment_endpoints(ref_poly, 10)

# pick a display length based on reference extents
xmin, xmax = np.min(ref_poly[:,0]), np.max(ref_poly[:,0])
zmin, zmax = np.min(ref_poly[:,1]), np.max(ref_poly[:,1])
disp_len = 0.6 * max(xmax - xmin, zmax - zmin)

p1 = M - n_hat * disp_len
p2 = M + n_hat * disp_len
plt.plot([p1[0], p2[0]], [p1[1], p2[1]], linestyle=":", linewidth=2.0, color="k", label="Midpoint line (seg 10)")


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

# =========================
# Interactive time slider: reveal trajectories up to time T
# =========================
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# --- Which drone is "embodied"? pull from JSON if recorded; else fallback by name ---
embodied_names = set()
if "trajectories" in data:
    for i, traj in enumerate(data["trajectories"]):
        if traj.get("isEmbodied") or traj.get("embodied") or traj.get("is_embodied"):
            embodied_names.add(traj.get("name", f"id:{traj.get('id', i)}"))
# Fallback heuristic
for n in list(drone_tracks.keys()):
    lo = n.lower()
    if "embodied" in lo or "player" in lo or "ego" in lo:
        embodied_names.add(n)

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
for poly in [coverage_poly_seg1, coverage_poly_seg2, coverage_poly_seg4]:
    if poly is None or Point is None:
        continue
    patches_here = shapely_to_patches(
        poly,
        alpha=0.25,
        facecolor='C1',
        edgecolor='none',
        zorder=1
    )
    for p in patches_here:
        p.set_visible(False)
        ax.add_patch(p)
        coverage_patches.append(p)

# Prepare a colored line for each drone and a per-drone endpoint marker
drone_lines = {}
drone_markers = {}
for name, d in drone_tracks.items():
    col = name_to_color.get(name, None)
    is_emb = name in embodied_names
    lw  = 3.2 if is_emb else 1.8
    alp = 1.0 if is_emb else 0.75
    line, = ax.plot([], [], linewidth=lw, alpha=alp, label=name, color=col)
    mark = ax.scatter([], [], s=(48 if is_emb else 28), marker='o', color=col if col else None, zorder=3)
    drone_lines[name] = line
    drone_markers[name] = mark

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

def on_btn_clicked(event):
    coverage_visible[0] = not coverage_visible[0]
    for p in coverage_patches:
        p.set_visible(coverage_visible[0])
    fig.canvas.draw_idle()

btn.on_clicked(on_btn_clicked)

# Legend (comment out if too many drones)
ax.legend(loc="best", fontsize=8, ncol=1)

plt.show()
