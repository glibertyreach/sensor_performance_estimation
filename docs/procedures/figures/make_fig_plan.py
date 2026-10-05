"""
fig_plan.png -- the default full plan: the target centers of every planned pose in the sensor
frame, side view (H against Z) and front view (H against V), colored by series, with the frustum of
the left camera over the working range.

The poses are not drawn by hand. They come from plan_full_session() with the default
CharacterizationParameters, the indicative SensorGeometry and a fixed seed, which is the plan
that the capture budget of the document is computed from (see build/build.py). Each point is the
reference point of the target (its center on the front face) at that pose.

What to see: the A stations in two columns of five field positions each across Z, the shape
stations (five depths) of B, C and D, the registration poses scattered through the volume, the
off-axis field poses and the phase-jitter clouds (small blobs around every station).

    python3 docs/procedures/figures/make_fig_plan.py      (from the repository root)
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import figfacts  # noqa: E402
from sensorperf.acquisition.plan import SERIES_LABELS, plan_full_session  # noqa: E402
from sensorperf.parameters import (  # noqa: E402
    CharacterizationParameters, PROCEDURE_AREA, PROCEDURE_DETECTION, PROCEDURE_EDGES, PROCEDURE_NOISE,
    PROCEDURE_REGISTRATION, PROCEDURE_SENTINEL, PROCEDURE_ZSTEP, SensorGeometry,
)

PARAMS = CharacterizationParameters()
GEOMETRY = SensorGeometry.indicative()
PLAN_SEED = 0
"""Seed of the generator (the same seed as the build's budget plan)."""

OUTPUT_DPI = 200
FIGURE_SIZE_IN = (13.0, 5.4)
FIGURE_NAME = "fig_plan"

# Okabe-Ito palette and one marker per series.
BLACK = "#000000"
ORANGE = "#E69F00"
SKY_BLUE = "#56B4E9"
BLUISH_GREEN = "#009E73"
BLUE = "#0072B2"
VERMILLION = "#D55E00"
REDDISH_PURPLE = "#CC79A7"
GRAY = "#595959"
SERIES_STYLE = {
    PROCEDURE_REGISTRATION: (SKY_BLUE, "D"),
    PROCEDURE_NOISE: (BLUE, "o"),
    PROCEDURE_EDGES: (BLUISH_GREEN, "s"),
    PROCEDURE_ZSTEP: (VERMILLION, "^"),
    PROCEDURE_AREA: (REDDISH_PURPLE, "v"),
    PROCEDURE_DETECTION: (ORANGE, "."),
    PROCEDURE_SENTINEL: (BLACK, "x"),
}
SERIES_DRAW_ORDER = tuple(reversed(list(SERIES_STYLE.items())))
"""Draw order: the many D poses first, so that the few poses of the other series stay visible on top."""
MARKER_SIZE = 8.0
MARKER_ALPHA = 0.55
FRUSTUM_COLOR = GRAY


def main() -> None:
    rng = np.random.default_rng(PLAN_SEED)
    plan = plan_full_session(PARAMS, GEOMETRY, rng)
    fig, (side, front) = plt.subplots(1, 2, figsize=FIGURE_SIZE_IN, dpi=OUTPUT_DPI,
                                      gridspec_kw={"width_ratios": [1.15, 1.0]})
    z_min, z_max = PARAMS.z_min_mm, PARAMS.z_max_mm
    half_w = GEOMETRY.image_width_px / 2.0 / GEOMETRY.sensor_fx_px
    half_h = GEOMETRY.image_height_px / 2.0 / GEOMETRY.sensor_fy_px
    # Frustum in the side view (H against Z), edges of the field of view from the camera center.
    side.plot([0, half_w * z_max, ], [0, z_max], color=FRUSTUM_COLOR, lw=1.0)
    side.plot([0, -half_w * z_max], [0, z_max], color=FRUSTUM_COLOR, lw=1.0)
    for z in (z_min, z_max):
        side.plot([-half_w * z, half_w * z], [z, z], color=FRUSTUM_COLOR, lw=0.8, linestyle=":")
    # Frustum cross-sections in the front view (H against V) at Z_MIN and Z_MAX.
    for z, style in ((z_min, "-"), (z_max, "--")):
        front.add_patch(Rectangle((-half_w * z, -half_h * z), 2 * half_w * z, 2 * half_h * z, facecolor="none",
                                  edgecolor=FRUSTUM_COLOR, lw=1.0, linestyle=style))
        front.text(half_w * z - 8, -half_h * z - 8, f"field at Z = {z:g} mm", fontsize=7, color=GRAY, ha="right",
                   va="bottom")
    counts = {}
    for rank, (procedure, (color, marker)) in enumerate(SERIES_DRAW_ORDER):
        members = [c for c in plan if c.procedure == procedure]
        if not members:
            continue
        counts[procedure] = len(members)
        centers = np.array([c.target_to_camera.translation for c in members])
        label = f"{SERIES_LABELS[procedure]} ({len(members)})"
        kw = dict(s=MARKER_SIZE, marker=marker, c=color, alpha=MARKER_ALPHA, linewidths=0.8)
        side.scatter(centers[:, 0], centers[:, 2], label=label, zorder=2 + rank, **kw)
        front.scatter(centers[:, 0], centers[:, 1], zorder=2 + rank, **kw)
    side.set_xlabel("camera H (mm)")
    side.set_ylabel("camera Z, depth (mm)")
    side.set_ylim(z_max + 60, -40)
    side.set_title("(a) side view: H against Z, with the frustum", fontsize=10)
    handles, labels = side.get_legend_handles_labels()
    order = sorted(range(len(labels)), key=lambda i: list(SERIES_STYLE).index(next(k for k in SERIES_STYLE if SERIES_LABELS[k] == labels[i].rsplit(" (", 1)[0])))
    fig.legend([handles[i] for i in order], [labels[i] for i in order], fontsize=8, loc="lower center", ncol=7, frameon=False, markerscale=1.6)
    side.set_aspect("equal")
    front.set_xlabel("camera H (mm)")
    front.set_ylabel("camera V (mm)")
    front.invert_yaxis()
    front.set_title("(b) front view: H against V", fontsize=10)
    front.set_aspect("equal")
    fig.subplots_adjust(left=0.06, right=0.99, top=0.93, bottom=0.17, wspace=0.14)
    out = Path(__file__).resolve().parent / f"{FIGURE_NAME}.png"
    fig.savefig(out, dpi=OUTPUT_DPI, facecolor="white")
    plt.close(fig)
    figfacts.emit(FIGURE_NAME, pose_count=len(plan), total_poses=len(plan), **{f"poses_series_{k}": v for k, v in counts.items()})
    print(f"wrote {out} ({len(plan)} poses)")


if __name__ == "__main__":
    main()
