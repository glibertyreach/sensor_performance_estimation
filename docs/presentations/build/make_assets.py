"""Make the figure panels and drawings used by the VSX3000 performance-testing decks from the procedure's figures.

Run from the repository root:
    python3 docs/presentations/build/make_assets.py

Reads  docs/procedures/figures/fig_*.png and docs/procedures/drawings/PT-*.png
Writes docs/presentations/assets/*.png

Whole figures and drawings are copied as they are (no cropping, no title removal). The one panel that is
cut out, the stations along Z (the left panel of fig_stations.png), is taken by a named box, its panel
title is erased, and the remaining content is cropped to its own bounds with a fixed margin restored on
every side (so that removing the title leaves no gap at the top and no title remains). Margins are filled
with the source figure's own background color (pure white). All numbers below are pixels of the source
figure, found by inspecting it.

Files in the assets folder that no content file references are removed (stage-1 leftovers). A drawing that
does not exist yet is skipped with a message; the deck generator then draws a labeled placeholder in its slot.
"""
import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image

REPO_ROOT = Path(__file__).resolve().parents[3]  # build/ -> presentations/ -> docs/ -> repository root
FIGURES_DIR = REPO_ROOT / "docs" / "procedures" / "figures"
DRAWINGS_DIR = REPO_ROOT / "docs" / "procedures" / "drawings"
PRESENTATIONS_DIR = REPO_ROOT / "docs" / "presentations"
ASSETS_DIR = PRESENTATIONS_DIR / "assets"
CONTENT_FILES = [  # the content files whose "image" and "images" fields name the assets in use
    PRESENTATIONS_DIR / "build" / "perf_build_deck_content.json",
    PRESENTATIONS_DIR / "build" / "perf_procedure_deck_content.json",
]

MARGIN_PX = 20  # white margin restored on every side of a cropped panel
CONTENT_THRESHOLD = 12  # a pixel is content when its RGB differs from the background by more than this (sum of channel differences)

# fig_stations.png (2600 x 1040): panel (a), the stations along Z, on the left; panel (b), the field positions, on the right.
STATIONS_SPLIT_X = 1386  # blank columns from x = 1380 to 1391 lie between the panels: (a) is left of it
STATIONS_A_TITLE_BOX = (0, 40, STATIONS_SPLIT_X, 100)  # (x0, y0, x1, y1) of title "(a) Z stations" (y = 45 to 70, x = 655 to 838); the first content, the tick labels, starts at y = 122

# Whole figures: asset name <- source figure (copied as it is).
WHOLE_FIGURES = {
    "targets.png": "fig_targets.png",
    "chamfer.png": "fig_chamfer.png",
    "mounting.png": "fig_mounting.png",
    "setup.png": "fig_setup.png",
    "plan_example.png": "fig_plan.png",
    "zstep.png": "fig_zstep.png",
}
# Drawings (3300 x 2100 px), copied unchanged under the same name.
DRAWINGS = [
    "PT-01_target_adapter.png",
    "PT-02_target_spigot.png",
    "PT-03_standoff_set.png",
    "PT-04_T3a_raised_square.png",
    "PT-05_T3b_square_window.png",
    "PT-06_T4_disk_plate_sheet1.png",
    "PT-07_T5_cutout_plate.png",
]


def background(img):
    """Background color of a figure: its top-left pixel (every source figure has an opaque background)."""
    return tuple(img.getpixel((0, 0))[:3])


def panel(img, x_range, title_box):
    """Cut a panel out of a figure: erase its title box, crop to the content bounds, restore the margins."""
    rgb = img.convert("RGB")
    bg = background(rgb)
    x0, x1 = x_range
    work = rgb.crop((x0, 0, x1, rgb.height))
    if title_box is not None:  # title_box is in source coordinates; shift it into the panel's
        tx0, ty0, tx1, ty1 = title_box
        work.paste(bg, (tx0 - x0, ty0, tx1 - x0, ty1))
    diff = np.abs(np.array(work).astype(int) - np.array(bg)).sum(axis=2) > CONTENT_THRESHOLD
    ys = np.where(diff.any(axis=1))[0]
    xs = np.where(diff.any(axis=0))[0]
    content = work.crop((int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1))
    out = Image.new("RGB", (content.width + 2 * MARGIN_PX, content.height + 2 * MARGIN_PX), bg)
    out.paste(content, (MARGIN_PX, MARGIN_PX))
    return out


def referenced_assets():
    """Asset file names ('assets/<name>') that the content files use."""
    names = set()
    for f in CONTENT_FILES:
        for s in json.loads(f.read_text(encoding="utf8"))["slides"]:
            for rel in [s.get("image"), s.get("image2")] + list(s.get("images", [])):
                if rel:
                    names.add(Path(rel).name)
    return names


def main():
    ASSETS_DIR.mkdir(parents=True, exist_ok=True)
    outputs = {}
    for name, src in WHOLE_FIGURES.items():
        shutil.copyfile(FIGURES_DIR / src, ASSETS_DIR / name)  # used whole, byte for byte
        with Image.open(FIGURES_DIR / src) as im:
            print(f"{name}: {im.width} x {im.height} (copied whole)")
    stations = Image.open(FIGURES_DIR / "fig_stations.png")
    outputs["stations_left.png"] = panel(stations, (0, STATIONS_SPLIT_X), STATIONS_A_TITLE_BOX)
    for name, img in outputs.items():
        img.save(ASSETS_DIR / name, optimize=True)
        print(f"{name}: {img.width} x {img.height}")
    for name in DRAWINGS:
        src = DRAWINGS_DIR / name
        if not src.exists():
            print(f"{name}: NOT FOUND in {DRAWINGS_DIR}, skipped (the deck shows a placeholder)")
            continue
        shutil.copyfile(src, ASSETS_DIR / name)
        with Image.open(src) as im:
            print(f"{name}: {im.width} x {im.height} (copied)")
    # remove stage-1 leftovers: any asset that no content file references
    used = referenced_assets()
    for f in sorted(ASSETS_DIR.iterdir()):
        if f.is_file() and f.name not in used:
            f.unlink()
            print(f"removed unreferenced {f.name}")
    missing = sorted(used - {f.name for f in ASSETS_DIR.iterdir()})
    if missing:
        print("referenced but absent: " + ", ".join(missing))


if __name__ == "__main__":
    main()
