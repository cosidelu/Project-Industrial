"""
Generate the 3D figures for the slides:
  1. slide_cylinder_filter.png  -> inspection positions + cylindrical filter
  2. slide_duplicate_filter.png -> detected defects + duplicate-filter radius

The script reads the real parameters from inspection_and_marking.py and
Variables.py, so the figures stay in sync with the tuning.

Run:  python slides_plots.py
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

import kinematics_v2 as kin
from Variables import HELMET_CENTER_GLOBAL
from defects_id_wrapper import duplicate_filter
from spherical_movement import variable_helmet_radius
from inspection_and_marking import (
    INSPECTION_RADIUS,
    INSPECTION_RADIUS_FN,
    GLOBAL_CYLINDER_RADIUS,
    GLOBAL_HEIGHT_RANGE,
    DUPLICATE_DISTANCE,
)

HC = np.array(HELMET_CENTER_GLOBAL, dtype=float)

# Inspection positions [r, alpha, beta] as actually set in
# PHASE2_inspection_marking_together.ipynb (7 shots, overriding the
# 4-position default of inspection_and_marking.py).
INSPECTION_POSITIONS = [
    [INSPECTION_RADIUS,   0, 90],   # top
    [INSPECTION_RADIUS,   0, 45],   # front / mid
    [INSPECTION_RADIUS,   0, 25],   # front / low
    [INSPECTION_RADIUS,  45, 45],   # right side
    [INSPECTION_RADIUS,  40, 80],   # right side, forward
    [INSPECTION_RADIUS, -40, 80],   # left side, forward
    [INSPECTION_RADIUS, -45, 45],   # left side
]

# Real unique defects from the PHASE2 notebook run: (pos3d_global, area).
REAL_DEFECTS = [
    ([43.1, 677.8, 387.7], 1529.0),
    ([-81.2, 706.9, 364.8], 3349.0),
    ([52.0, 829.0, 205.1], 1088.5),
    ([109.6, 729.9, 326.3], 1610.5),
]

# Helmet approximated as a sphere: radius 180 mm, centre 70 mm above
# HELMET_CENTER_GLOBAL (the apex, beta=90, lies along +Z in this convention).
HELMET_SPHERE_RADIUS = 180.0
HELMET_SPHERE_CENTER = HC + np.array([0.0, 0.0, 70.0])


# ----------------------------------------------------------------------
# Geometry helpers
# ----------------------------------------------------------------------
def sphere_surface(center, radius, n=30):
    u = np.linspace(0, 2 * np.pi, n)
    v = np.linspace(0, np.pi, n)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    return x, y, z


def cylinder_mask(X, Y, Z, p_start, axis_dir, r_min, r_max, radius):
    """Boolean mask (same shape as X) of the sphere-surface points that fall
    inside the camera-frame cylinder: axial depth within [r_min, r_max] and
    lateral distance <= radius. Replicates the cam_cylinder_filter test."""
    axis_dir = np.asarray(axis_dir, float)
    axis_dir = axis_dir / np.linalg.norm(axis_dir)
    rx, ry, rz = X - p_start[0], Y - p_start[1], Z - p_start[2]
    depth = rx * axis_dir[0] + ry * axis_dir[1] + rz * axis_dir[2]
    lx = rx - depth * axis_dir[0]
    ly = ry - depth * axis_dir[1]
    lz = rz - depth * axis_dir[2]
    lateral = np.sqrt(lx**2 + ly**2 + lz**2)
    return (depth >= r_min) & (depth <= r_max) & (lateral <= radius)


def set_equal_3d(ax, pts, center=None):
    pts = np.asarray(pts)
    # Centre the view on `center` (the helmet) if given, else on the points.
    c = np.asarray(center, float) if center is not None else pts.mean(axis=0)
    # Span large enough to contain every point around that centre.
    span = np.abs(pts - c).max()
    ax.set_xlim(c[0] - span, c[0] + span)
    ax.set_ylim(c[1] - span, c[1] + span)
    ax.set_zlim(c[2] - span, c[2] + span)
    ax.set_box_aspect((1, 1, 1))  # equal scale on all axes -> 1:1:1
    ax.set_xlabel("X [mm]"); ax.set_ylabel("Y [mm]"); ax.set_zlabel("Z [mm]")


# ----------------------------------------------------------------------
# PLOT 1 - cylindrical filter
# ----------------------------------------------------------------------
def draw_cylinder_scene(ax):
    # Helmet sphere (light wireframe for context).
    sx, sy, sz = sphere_surface(HELMET_SPHERE_CENTER, HELMET_SPHERE_RADIUS, n=28)
    ax.plot_wireframe(sx, sy, sz, color="0.75", linewidth=0.3, alpha=0.35)
    ax.scatter(*HELMET_SPHERE_CENTER, color="black", s=25)

    all_pts = [HELMET_SPHERE_CENTER + HELMET_SPHERE_RADIUS,
               HELMET_SPHERE_CENTER - HELMET_SPHERE_RADIUS]

    # Dense sphere surface mesh; per camera we keep only the part inside the
    # cylindrical filter (NaN elsewhere) and draw it as a translucent surface,
    # so overlaps between cameras actually show through.
    SX, SY, SZ = sphere_surface(HELMET_SPHERE_CENTER, HELMET_SPHERE_RADIUS, n=160)

    cmap = plt.cm.turbo(np.linspace(0.05, 0.95, len(INSPECTION_POSITIONS)))
    for i, (sph, col) in enumerate(zip(INSPECTION_POSITIONS, cmap), start=1):
        _, alpha, beta = sph
        # The capture radius is NOT constant: point_and_shoot passes
        # variable_helmet_radius (ellipsoidal working surface), so evaluate it
        # per (alpha, beta) to place the camera where it really sits.
        r_cap = INSPECTION_RADIUS_FN(alpha, beta)
        cam_pos, _ = kin.to_helmet_coordinates([r_cap, alpha, beta],
                                               HELMET_CENTER_GLOBAL)
        # The cylindrical filter lives in the camera frame; its axis is the
        # optical Z axis, pointing from the camera towards the helmet centre.
        axis_dir = HC - cam_pos

        mask = cylinder_mask(
            SX, SY, SZ, cam_pos, axis_dir,
            r_min=GLOBAL_HEIGHT_RANGE[0],
            r_max=GLOBAL_HEIGHT_RANGE[1],
            radius=GLOBAL_CYLINDER_RADIUS,
        )
        if mask.any():
            Xm = np.where(mask, SX, np.nan)
            Ym = np.where(mask, SY, np.nan)
            Zm = np.where(mask, SZ, np.nan)
            ax.plot_surface(Xm, Ym, Zm, color=col, alpha=0.4,
                            linewidth=0, shade=False, antialiased=False)
        # proxy artist for the legend (plot_surface has no label support)
        ax.plot([], [], [], color=col, lw=6, alpha=0.5,
                label=f"{i}: a={alpha:.0f}, b={beta:.0f} (r={r_cap:.0f})")
        # camera position + optical axis line
        ax.scatter(*cam_pos, color=col, s=55, edgecolor="black",
                   depthshade=False)
        ax.text(*cam_pos, f"  {i}", fontsize=10, color="black", weight="bold")
        line = np.vstack([cam_pos, HC])
        ax.plot(line[:, 0], line[:, 1], line[:, 2],
                color=col, lw=0.6, ls="--", alpha=0.4)
        all_pts.append(cam_pos)

    ax.set_title(
        "Sphere-cylinder intersection per inspection position\n"
        f"cylinder radius = {GLOBAL_CYLINDER_RADIUS:.0f} mm, "
        f"depth range = {GLOBAL_HEIGHT_RANGE[0]:.0f}-{GLOBAL_HEIGHT_RANGE[1]:.0f} mm"
    )
    ax.legend(loc="upper left", fontsize=8,
              title="capture (variable radius)")
    set_equal_3d(ax, all_pts, center=HELMET_SPHERE_CENTER)


# ----------------------------------------------------------------------
# PLOT 2 - duplicate filter (runs the real duplicate_filter)
# ----------------------------------------------------------------------
class FakeDefect:
    """Minimal stand-in exposing the attributes duplicate_filter uses."""
    def __init__(self, pos3d_global, area):
        self.pos3d_global = np.asarray(pos3d_global, float)
        self.area = area


def draw_duplicate_scene(ax):
    # Start from the 4 real unique defects of the notebook run, then add one
    # near-duplicate (within DUPLICATE_DISTANCE) so the 5 -> 4 merge of the
    # real run is reproduced. The duplicate has a smaller area than its twin,
    # so duplicate_filter keeps the real (larger-area) detection.
    detections = [FakeDefect(p, a) for p, a in REAL_DEFECTS]
    twin = detections[1]                       # largest area defect (3349)
    dup_pos = twin.pos3d_global + np.array([6.0, -5.0, 7.0])  # ~10.5 mm away
    detections.insert(2, FakeDefect(dup_pos, area=twin.area * 0.4))

    unique = duplicate_filter(detections, distance_threshold=DUPLICATE_DISTANCE)
    unique_ids = {id(u) for u in unique}

    kept = np.array([d.pos3d_global for d in detections if id(d) in unique_ids])
    removed = np.array([d.pos3d_global for d in detections if id(d) not in unique_ids])

    sx, sy, sz = sphere_surface(HELMET_SPHERE_CENTER, HELMET_SPHERE_RADIUS, n=24)
    ax.plot_wireframe(sx, sy, sz, color="0.7", linewidth=0.3, alpha=0.35)

    # Merge sphere (radius = DUPLICATE_DISTANCE) only around the kept defects:
    # any detection inside is treated as a duplicate and discarded.
    for k in kept:
        dx, dy, dz = sphere_surface(k, DUPLICATE_DISTANCE, n=14)
        ax.plot_wireframe(dx, dy, dz, color="tab:blue", linewidth=0.4, alpha=0.45)

    ax.scatter(kept[:, 0], kept[:, 1], kept[:, 2],
               color="tab:green", s=80, edgecolor="black", depthshade=False,
               label="kept (largest area)")
    if len(removed):
        ax.scatter(removed[:, 0], removed[:, 1], removed[:, 2],
                   color="tab:red", s=130, marker="X", edgecolor="black",
                   depthshade=False, linewidth=1.5, zorder=10,
                   label="discarded duplicate")
        # connect each discarded duplicate to the kept defect it merged into
        for r in removed:
            nearest = kept[np.argmin(np.linalg.norm(kept - r, axis=1))]
            seg = np.vstack([r, nearest])
            ax.plot(seg[:, 0], seg[:, 1], seg[:, 2],
                    color="tab:red", lw=1.2, ls="--", zorder=9)
        ax.text(*removed[0], "  within 15 mm\n  -> merged", fontsize=8,
                color="tab:red")

    ax.set_title(
        "Detected defects and duplicate-filter radius\n"
        f"merge threshold = {DUPLICATE_DISTANCE:.0f} mm  "
        f"({len(detections)} detections -> {len(unique)} unique)"
    )
    ax.legend(loc="upper right")
    pts = np.vstack([[d.pos3d_global for d in detections],
                     HELMET_SPHERE_CENTER + HELMET_SPHERE_RADIUS,
                     HELMET_SPHERE_CENTER - HELMET_SPHERE_RADIUS])
    set_equal_3d(ax, pts, center=HELMET_SPHERE_CENTER)


# ----------------------------------------------------------------------
# Rendering: transparent PNG + rotating transparent GIF
# ----------------------------------------------------------------------
def _rgba_to_transparent_p(im):
    """Convert an RGBA frame to a paletted image with a transparent index,
    so the GIF keeps a see-through background while rotating."""
    from PIL import Image
    alpha = im.getchannel("A")
    p = im.convert("RGB").convert("P", palette=Image.ADAPTIVE, colors=255)
    # fully/mostly transparent pixels -> reserved palette index 255
    mask = alpha.point(lambda a: 255 if a <= 128 else 0)
    p.paste(255, mask)
    return p


def render_scene(draw_fn, name, elev=20, n_frames=120, duration=110, dpi=130):
    """Draw the scene once, save a transparent PNG, then spin the azimuth
    360 deg and save a transparent rotating GIF."""
    import io
    from PIL import Image

    fig = plt.figure(figsize=(8, 8))
    fig.patch.set_alpha(0.0)                      # transparent figure
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("none")                      # transparent axes panel
    draw_fn(ax)

    # static PNG (transparent)
    ax.view_init(elev=elev, azim=-72)
    fig.savefig(f"{name}.png", dpi=200, transparent=True)
    print(f"saved {name}.png")

    # rotating GIF (transparent)
    frames = []
    for az in np.linspace(0, 360, n_frames, endpoint=False):
        ax.view_init(elev=elev, azim=az)
        buf = io.BytesIO()
        fig.savefig(buf, format="png", dpi=dpi, transparent=True)
        buf.seek(0)
        frames.append(_rgba_to_transparent_p(Image.open(buf).convert("RGBA")))

    frames[0].save(
        f"{name}.gif", save_all=True, append_images=frames[1:],
        duration=duration, loop=0, transparency=255, disposal=2, optimize=False,
    )
    print(f"saved {name}.gif")
    plt.close(fig)


if __name__ == "__main__":
    render_scene(draw_cylinder_scene, "slide_cylinder_filter", elev=22)
    render_scene(draw_duplicate_scene, "slide_duplicate_filter", elev=18)
