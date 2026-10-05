#!/usr/bin/env python3
"""
The single builder of the performance-testing procedure document.

    python3 docs/procedures/build/build.py            # figures, Markdown, docx, manifest, gate, baseline
    python3 docs/procedures/build/build.py --no-docx  # Markdown and manifest only (no pandoc needed)
    python3 docs/procedures/build/build.py --intended "5" "Figure Index"   # gate the scope of a review edit

Source of truth
    docs/procedures/performance_test_procedure.template.md   the text, with placeholders
    sensorperf/parameters.py                                  every number the text quotes
    sensorperf/acquisition/plan.py                            the pose and frame counts (capture budget)
    docs/procedures/figures/make_fig_*.py                     every figure (regenerated on each build)

Placeholders in the template (all are replaced; an unknown one fails the build):
    {{PARAMETER_TABLE}}            Section 2 table generated from CharacterizationParameters
    {{BUDGET_TABLE}}               Section 9 capture budget computed from the default plan
    {{FIGURE_INDEX}}               the list of every figure in the document, generated
    {{FILE_TABLE}}                 the package files with line counts (software appendix)
    {{CLI_HELP:<module>}}          the --help output of python3 -m sensorperf.cli.<module>
    {{VALUE:<field>}}              a CharacterizationParameters field, formatted
    {{DERIVED:<name>}}             a derived quantity from derived_values() (counts, station lists, totals, dates)

Outputs
    docs/procedures/performance_test_procedure.md / .docx
    docs/procedures/Review/manifest.json      elements (sections, figures, tables) + invariants with phrases
    docs/procedures/Review/progress.json      review ledger (seeded once, never overwritten)
    docs/procedures/Review/baselines/<date>_<n>/   dated baseline (docx, md, manifest) after each build,
                                                  the convention of the flexible_plane_fit specification
    the gate report of build/verify_doc.py (exit status 1 when it fails)

Why a builder: the document quotes dozens of counts (stations, poses, frames,
rungs, hours). Typing them by hand lets the text and the code drift apart; here
each is computed from the same parameters the code runs with, so a change of a
parameter changes the document on the next build, and the verification gate
checks that every place that quotes a count agrees with the invariant.
"""
from __future__ import annotations

import argparse
import datetime as dt
import importlib
import io
import json
import re
import shutil
import subprocess
import sys
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROCEDURES_DIR = HERE.parent
REPO_ROOT = PROCEDURES_DIR.parent.parent
FIGURES_DIR = PROCEDURES_DIR / "figures"
REVIEW_DIR = PROCEDURES_DIR / "Review"
BASELINES_DIR = REVIEW_DIR / "baselines"
TEMPLATE_PATH = PROCEDURES_DIR / "performance_test_procedure.template.md"
MARKDOWN_PATH = PROCEDURES_DIR / "performance_test_procedure.md"
DOCX_PATH = PROCEDURES_DIR / "performance_test_procedure.docx"
MANIFEST_PATH = REVIEW_DIR / "manifest.json"
PROGRESS_PATH = REVIEW_DIR / "progress.json"
DOCUMENT_TITLE = "VSX3000 Sensor Performance Testing: Step-by-Step Procedure"
LUA_FILTER = HERE / "image_width.lua"
GATE_SCRIPT = HERE / "verify_doc.py"

PLACEHOLDER_PATTERN = re.compile(r"\{\{([A-Z_]+)(?::([A-Za-z0-9_.\-]+))?\}\}")
FIGURE_PATTERN = re.compile(r"^!\[(?P<alt>[^\]]*)\]\((?P<path>figures/[^)]+)\)\s*$", re.MULTILINE)
CAPTION_PATTERN = re.compile(r"^(Figure|Table) (?P<num>[0-9]+[a-z]?)\.\s+(?P<text>.+)$", re.MULTILINE)
HEADING_PATTERN = re.compile(r"^(#{1,3})\s+(?P<text>.+?)\s*$", re.MULTILINE)
SECTION_NUMBER_PATTERN = re.compile(r"^(?:Appendix\s+)?(?P<num>[0-9]+(?:\.[0-9]+)*|[A-Z])[.\s]")

sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(FIGURES_DIR))
import figfacts  # noqa: E402  (feature_diameters_text: the figure and the text format the diameters alike)

from sensorperf.acquisition.plan import MIN_POSE_LOG_DECIMALS  # noqa: E402
from sensorperf.parameters import CharacterizationParameters, SensorGeometry, parameter_table_rows  # noqa: E402

BUDGET_FRAME_RATE_HZ = 10.0
"""Frame rate the capture budget assumes (Section 9: 'assumes 10 frames/s'); replaced by the
measured frame rate once Step 4.5 fills in SensorGeometry.frame_rate_hz."""


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------
def regenerate_figures() -> list[Path]:
    """Run every figures/make_fig_*.py (each writes its PNG and a .facts.json sidecar)."""
    written = []
    for script in sorted(FIGURES_DIR.glob("make_fig_*.py")):
        print(f"figure: {script.name}")
        subprocess.run([sys.executable, str(script)], check=True, cwd=str(REPO_ROOT))
        written.append(FIGURES_DIR / (script.stem.replace("make_", "") + ".png"))
    return written


# ---------------------------------------------------------------------------
# Generated tables and derived values
# ---------------------------------------------------------------------------
def parameter_table(params: CharacterizationParameters) -> str:
    rows = parameter_table_rows(params)
    lines = ["| Name | Value | Meaning |", "|---|---|---|"]
    for name, value, meaning in rows:
        lines.append(f"| `{name}` | {value} | {meaning} |")
    return "\n".join(lines)


def default_plan_and_budget(params: CharacterizationParameters, geometry: SensorGeometry):
    """The default full plan and its budget rows, from sensorperf.acquisition.plan."""
    import numpy as np
    from sensorperf.acquisition import plan as planning
    rng = np.random.default_rng(0)
    plan = planning.plan_full_session(params, geometry, rng)
    budget = budget_rows(plan, BUDGET_FRAME_RATE_HZ, params.move_and_settle_time_s)
    return plan, budget


BUDGET_ROWS = (
    ("Registration", {"R": None}),
    ("A main + tilt + sentinels", {"A": None, "S": None}),
    ("B-HV edges", {"B": None}),
    ("B-Z ladder + staircase", {"Z": None}),
    ("C main + field", {"C": None}),
    ("D main", {"D": ("jitter",)}),
    ("D extended (0% series)", {"D": ("extended",)}),
)
"""The rows of Section 9 of the specification: label and {procedure letter: sub-series or None for all}.
The robot time of a row is (poses x move-and-settle time + frames / frame rate) / 3600, the planner's own rule
(sensorperf.acquisition.plan.capture_budget); the totals equal the planner's."""


def budget_rows(plan, frame_rate_hz: float, move_settle_s: float) -> list[tuple[str, int, int, float]]:
    """(label, poses, frames, robot hours) per row of BUDGET_ROWS, from the default plan."""
    rows = []
    for label, members in BUDGET_ROWS:
        chosen = [c for c in plan if c.procedure in members
                  and (members[c.procedure] is None or c.subseries in members[c.procedure])]
        if label == "C main + field" and any(c.subseries == "open" for c in chosen):
            label = "C main + field + open"
        poses = len(chosen)
        frames = sum(c.frames for c in chosen)
        rows.append((label, poses, frames, (poses * move_settle_s + frames / frame_rate_hz) / 3600.0))
    return rows


def budget_table(budget) -> tuple[str, dict]:
    """Markdown table of the capture budget and its totals."""
    lines = ["| Series | Poses | Frames | Robot time (h) |", "|---|---:|---:|---:|"]
    total_poses = total_frames = 0
    total_hours = 0.0
    for label, poses, frames, hours in budget:
        lines.append(f"| {label} | {poses:,} | {frames:,} | {hours:.2f} |")
        total_poses += poses
        total_frames += frames
        total_hours += hours
    lines.append(f"| **Total** | **{total_poses:,}** | **{total_frames:,}** | **{total_hours:.1f}** |")
    return "\n".join(lines), {"total_poses": total_poses, "total_frames": total_frames,
                              "total_robot_hours": round(total_hours, 1)}


def file_table() -> str:
    """The package files and their line counts (software appendix), generated so it cannot go stale."""
    lines = ["| File | Lines | What it does |", "|---|---:|---|"]
    for path in sorted((REPO_ROOT / "sensorperf").rglob("*.py")):
        text = path.read_text(encoding="utf-8")
        count = text.count("\n")
        summary = _first_docstring_line(text)
        lines.append(f"| `{path.relative_to(REPO_ROOT).as_posix()}` | {count} | {summary} |")
    return "\n".join(lines)


def _first_docstring_line(text: str) -> str:
    match = re.search(r'"""\s*(.+?)(?:\n\n|""")', text, re.DOTALL)
    if not match:
        return ""
    return " ".join(match.group(1).split())[:160]


def cli_help(module: str) -> str:
    """The --help output of a command-line tool, captured in-process."""
    mod = importlib.import_module(f"sensorperf.cli.{module}")
    buffer = io.StringIO()
    saved_argv0 = sys.argv[0]
    sys.argv[0] = f"python3 -m sensorperf.cli.{module}"       # argparse prints this as the program name
    try:
        with redirect_stdout(buffer), redirect_stderr(buffer):
            mod.main(["--help"])
    except SystemExit:
        pass
    finally:
        sys.argv[0] = saved_argv0
    return buffer.getvalue().rstrip()


def _list_text(values) -> str:
    """Station lists as the document quotes them: 400, 476, 566 (whole millimeters)."""
    return ", ".join(f"{v:g}" for v in values)


def _and_text(values) -> str:
    """1131, 1345 and 1600."""
    texts = [f"{v:g}" for v in values]
    return texts[0] if len(texts) == 1 else ", ".join(texts[:-1]) + " and " + texts[-1]


def derived_values(params: CharacterizationParameters, geometry: SensorGeometry, budget_totals: dict) -> dict:
    ladder = params.z_stations_mm()
    shape = params.z_shape_stations_mm()
    reduced = params.z_reduced_stations_mm()
    zero = params.detection_zero_stations_mm()
    legacy = tuple(z for z in params.noise_stations_mm() if z not in ladder)
    diameters = params.feature_diameters_mm(geometry)

    def px_range(diameter: float) -> str:
        low = geometry.diameter_in_pixels(diameter, params.z_max_mm)
        high = geometry.diameter_in_pixels(diameter, params.z_min_mm)
        return f"{low:.2g} to {high:.2g}"

    values = {
        "build_date": dt.date.today().isoformat(),
        "z_min_mm": int(params.z_min_mm),
        "z_max_mm": int(params.z_max_mm),
        "station_count": len(ladder),
        "shape_station_count": len(shape),
        "reduced_station_count": len(reduced),
        "zero_station_count": len(zero),
        "legacy_depth_count": len(legacy),
        "ladder_text": _list_text(ladder),
        "shape_stations_text": _list_text(shape),
        "reduced_stations_text": _list_text(reduced),
        "zero_stations_text": _and_text(zero),
        "legacy_depths_text": _and_text(legacy),
        "station_ratio_text": "2^(1/4)" if abs(params.z_station_ratio - 2.0 ** 0.25) < 1e-12 else f"{params.z_station_ratio:.4g}",
        "field_position_count": 5,
        "tilt_count": len(params.tilt_angles_deg),
        "feature_count": params.feature_count,
        "level_count": params.feature_count * len(ladder),
        "feature_diameters": figfacts.feature_diameters_text(diameters),
        "feature_px_ranges_text": ", ".join(px_range(d) for d in diameters[:-1]) + f" and {px_range(diameters[-1])} px",
        "feature_min_diameter_mm": f"{diameters[0]:.1f}",
        "feature_ratio_text": "2 sqrt(2)" if abs(params.feature_ladder_ratio - 2.0 * 2.0 ** 0.5) < 1e-12 else f"{params.feature_ladder_ratio:.4g}",
        "blank_site_count": params.blank_sites_per_plate,
        "post_site_count": params.post_sites_per_plate,
        "isolation_mm_at_z_max": f"{params.feature_isolation_px * geometry.pixel_footprint_mm(params.z_max_mm):.0f}",
        "detection_trials_per_level": params.detection_trials_per_level,
        "detection_zero_trials": params.detection_zero_trials,
        "zstep_rung_count": len(params.z_step_ladder_mm),
        "zstep_smallest_mm": f"{min(params.z_step_ladder_mm):g}",
        "zstep_largest_mm": f"{max(params.z_step_ladder_mm):g}",
        "zero_bound_percent": f"{100.0 * params.rule_of_three_bound(params.detection_zero_trials):.1f}",
        "indicative_fx_px": f"{geometry.sensor_fx_px:.0f}",
        "noise_station_frames": params.frames_per_noise_station,
        "sentinel_frames": params.sentinel_frames,
        "budget_frame_rate_hz": f"{BUDGET_FRAME_RATE_HZ:g}",
        "pose_log_min_decimals": MIN_POSE_LOG_DECIMALS,
    }
    values.update(budget_totals)
    values["total_poses_text"] = f"{budget_totals['total_poses']:,}"
    values["total_frames_text"] = f"{budget_totals['total_frames']:,}"
    return values


def format_parameter(value) -> str:
    if isinstance(value, tuple):
        return ", ".join(format_parameter(v) for v in value)
    if isinstance(value, float):
        return str(int(value)) if value == int(value) else f"{value:g}"
    return str(value)


# ---------------------------------------------------------------------------
# Template rendering
# ---------------------------------------------------------------------------
def render(template: str, params: CharacterizationParameters, geometry: SensorGeometry, budget, derived: dict) -> str:
    budget_md, _ = budget_table(budget)
    generated = {
        "PARAMETER_TABLE": parameter_table(params),
        "BUDGET_TABLE": budget_md,
        "FILE_TABLE": file_table(),
    }

    def substitute(match: re.Match) -> str:
        kind, argument = match.group(1), match.group(2)
        if kind in generated and argument is None:
            return generated[kind]
        if kind == "FIGURE_INDEX":
            return "{{FIGURE_INDEX}}"                      # filled after the figures are known
        if kind == "CLI_HELP" and argument:
            return "```\n" + cli_help(argument) + "\n```"
        if kind == "VALUE" and argument:
            if not hasattr(params, argument):
                raise KeyError(f"template asks for unknown parameter {{{{VALUE:{argument}}}}}")
            return format_parameter(getattr(params, argument))
        if kind == "DERIVED" and argument:
            if argument not in derived:
                raise KeyError(f"template asks for unknown derived value {{{{DERIVED:{argument}}}}}")
            return str(derived[argument])
        raise KeyError(f"unknown placeholder {match.group(0)}")

    text = PLACEHOLDER_PATTERN.sub(substitute, template)
    text = text.replace("{{FIGURE_INDEX}}", figure_index(text))
    return text


def figures_in(text: str) -> list[tuple[str, str, str]]:
    """(number, caption, path) of every figure: an image line followed (within a few lines)
    by a 'Figure N. caption' line."""
    figures = []
    lines = text.splitlines()
    for index, line in enumerate(lines):
        image = FIGURE_PATTERN.match(line)
        if not image:
            continue
        caption_number, caption_text = "", ""
        for follow in lines[index + 1:index + 4]:
            caption = CAPTION_PATTERN.match(follow.strip())
            if caption and caption.group(1) == "Figure":
                caption_number, caption_text = caption.group("num"), caption.group("text")
                break
        figures.append((caption_number, caption_text, image.group("path")))
    return figures


def figure_index(text: str) -> str:
    # The first cell is the bare figure number: the gate's figure-index check keys its rows on it.
    lines = ["| Figure | Caption | File |", "|---|---|---|"]
    for number, caption, path in figures_in(text):
        lines.append(f"| {number} | {caption} | `{path}` |")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Review manifest and ledger
# ---------------------------------------------------------------------------
INVARIANTS = [
    {"name": "Station count of the ladder agrees everywhere", "key": "station_count", "phrase": "stations of the ladder"},
    {"name": "Shape station count agrees everywhere", "key": "shape_station_count", "phrase": "shape station"},
    {"name": "Reduced station count agrees everywhere", "key": "reduced_station_count", "phrase": "reduced station"},
    {"name": "Count of the farthest (extended-trial) stations agrees everywhere", "key": "zero_station_count",
     "phrase": "farthest station"},
    {"name": "Field position count agrees everywhere", "key": "field_position_count", "phrase": "field position"},
    {"name": "Features per plate agree everywhere", "key": "feature_count", "phrase": "features per plate"},
    {"name": "Feature diameters agree everywhere", "key": "feature_diameters", "pattern": "{value}"},
    {"name": "Blank sites per plate agree everywhere", "key": "blank_site_count", "phrase": "blank sites per plate"},
    {"name": "Z-step rung count agrees everywhere", "key": "zstep_rung_count", "phrase": "step size"},
    {"name": "Trials per feature and station agree everywhere", "key": "detection_trials_per_level",
     "phrase": "trials per feature and station"},
    {"name": "Extended-series trials agree everywhere", "key": "detection_zero_trials", "pattern": "{value} trials"},
    {"name": "Working range minimum agrees everywhere", "key": "z_min_mm", "pattern": "{value} mm"},
    {"name": "Working range maximum agrees everywhere", "key": "z_max_mm", "pattern": "{value} mm"},
    {"name": "Total planned poses agree everywhere", "key": "total_poses"},
    {"name": "Total planned frames agree everywhere", "key": "total_frames"},
    {"name": "Total robot hours agree everywhere", "key": "total_robot_hours"},
    {"name": "Frames per noise station agree everywhere", "key": "noise_station_frames"},
    {"name": "Sentinel frames agree everywhere", "key": "sentinel_frames"},
]
"""The shared numbers the gate proves consistent. A phrase (the counted noun) turns on the prose-drift check of the
gate; a pattern ({value} is replaced by the value) only records which sections quote the number."""

STALE_TERMS = (
    (r"\bT1\b", "the registration plate T1 no longer exists"),
    (r"ChArUco|circle-grid", "no pattern plate"),
    (r"fiducial", "registration is by plane correspondence on T2"),
    (r"T4-S|T4-L|T5-S|T5-L", "one disk plate T4 and one cutout plate T5"),
    (r"500 to 1000|500\u20131000|500-1000", "the working range is 400 to 1600 mm"),
    (r"\b11 (?:Z |noise )?stations", "one ladder of 9 stations"),
    (r"\b17\b (?:diameter|rung|level)", "the diameter ladder of 17 rungs is gone"),
    (r"DIAMETER_LADDER|DETECTION_LEVELS|DETECTION_FINE_LADDER", "removed parameters"),
    (r"pilot D_?50|pilot D_?0", "no pilot-based level selection"),
    (r"continuous-angle", "the continuous-angle variant is removed"),
    (r"ambient IR|IR light meter", "the laboratory is enclosed with constant lighting"),
    (r"enclosure|blackout", "the laboratory is enclosed with constant lighting"),
    (r"\b750 mm|Z0750", "no station at 750 mm"),
    (r"\bdial\b", "the dial indicator is not used"),
)
"""Terms of the old design that must not survive in the rendered Markdown (checked at every build)."""


ALLOWED_STALE_LINES = (
    r"Design change: the specification of 2026-10-04 used a dial indicator",   # history of the Z-step truth, section 8
    r"\{fiducial,planes\}|no pattern needed\); fiducial:",                      # the register tool's own --help text
)
"""Lines where a stale term is legitimate, with the reason in the comment (reported in the build summary)."""


def stale_term_hits(text: str) -> list[str]:
    hits = []
    for number, line in enumerate(text.splitlines(), start=1):
        if any(re.search(allowed, line) for allowed in ALLOWED_STALE_LINES):
            continue
        for pattern, why in STALE_TERMS:
            for match in re.finditer(pattern, line):
                hits.append(f"line {number}: {match.group(0)!r} ({why}): {line.strip()[:110]}")
    return hits


def section_elements(text: str, derived: dict) -> list[dict]:
    """One manifest element per heading, figure and table caption; sections assert the
    invariants whose derived value they quote (a number followed by the phrase, or the
    totals in the budget section)."""
    elements = []
    headings = list(HEADING_PATTERN.finditer(text))
    for index, match in enumerate(headings):
        title = match.group("text").strip()
        number = SECTION_NUMBER_PATTERN.match(title)
        key = f"§{number.group('num')}" if number else title
        body_end = headings[index + 1].start() if index + 1 < len(headings) else len(text)
        body = text[match.end():body_end]
        asserts = {}
        for invariant in INVARIANTS:
            value = derived.get(invariant["key"])
            phrase = invariant.get("phrase")
            pattern = invariant.get("pattern")
            if phrase and re.search(rf"\b{value}\b[^.\n]{{0,40}}{re.escape(phrase)}", body):
                asserts[invariant["key"]] = value
            elif pattern and re.search(rf"(?<![\d.]){re.escape(pattern.format(value=value))}", body):
                asserts[invariant["key"]] = value
        if "{{BUDGET" in body or "Robot time (h)" in body:
            for key_name in ("total_poses", "total_frames", "total_robot_hours"):
                asserts[key_name] = derived[key_name]
        refs = sorted(set(re.findall(r"Figure [0-9]+[a-z]?|Table [0-9]+|§[0-9]+(?:\.[0-9]+)*", body)))
        elements.append({"id": key, "kind": "section", "source": TEMPLATE_PATH.relative_to(PROCEDURES_DIR).as_posix(),
                         "title": title, "asserts": asserts, "refs": refs})
    for number, caption, path in figures_in(text):
        stem = Path(path).stem
        facts_path = FIGURES_DIR / f"{stem}.facts.json"
        element = {"id": f"Figure {number}", "kind": "figure", "source": f"figures/make_{stem}.py",
                   "caption": caption, "asserts": {}, "refs": []}
        if facts_path.exists():
            facts = json.loads(facts_path.read_text(encoding="utf-8"))
            element["facts"] = f"figures/{stem}.facts.json"
            element["asserts"] = {k: v for k, v in facts.items() if k in {i["key"] for i in INVARIANTS}}
        elements.append(element)
    for match in CAPTION_PATTERN.finditer(text):
        if match.group(1) == "Table":
            elements.append({"id": f"Table {match.group('num')}", "kind": "table",
                             "source": TEMPLATE_PATH.relative_to(PROCEDURES_DIR).as_posix(),
                             "caption": match.group("text"), "asserts": {}, "refs": []})
    return elements


def write_manifest(text: str, derived: dict) -> None:
    REVIEW_DIR.mkdir(exist_ok=True)
    manifest = {"document": DOCX_PATH.name, "built": dt.datetime.now().isoformat(timespec="seconds"),
                "invariant_values": {i["key"]: derived[i["key"]] for i in INVARIANTS},
                "elements": section_elements(text, derived), "invariants": INVARIANTS}
    MANIFEST_PATH.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    if not PROGRESS_PATH.exists():
        ledger = {e["id"]: {"reviewed": None, "tier": None, "note": "not yet reviewed"}
                  for e in manifest["elements"] if e["kind"] == "section"}
        PROGRESS_PATH.write_text(json.dumps(ledger, indent=2), encoding="utf-8")


# ---------------------------------------------------------------------------
# docx, baseline, gate
# ---------------------------------------------------------------------------
def build_docx() -> Path:
    """Markdown -> docx with pandoc (pypandoc), the image-width filter and a TOC; then declare
    the PNG content type pandoc omits (copied from the calibration repository's build_docx.sh)."""
    import pypandoc
    pypandoc.convert_file(str(MARKDOWN_PATH), "docx", outputfile=str(DOCX_PATH),
                          extra_args=[f"--resource-path={PROCEDURES_DIR}", f"--lua-filter={LUA_FILTER}",
                                      "--metadata", f"title={DOCUMENT_TITLE}", "--toc", "--toc-depth=1"])
    import zipfile
    tmp = DOCX_PATH.with_suffix(".docx.tmp")
    with zipfile.ZipFile(DOCX_PATH) as zin:
        content_types = zin.read("[Content_Types].xml").decode()
        if 'Extension="png"' not in content_types:
            content_types = content_types.replace(
                "<Default ", '<Default Extension="png" ContentType="image/png"/><Default ', 1)
            with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as zout:
                for item in zin.infolist():
                    data = content_types.encode() if item.filename == "[Content_Types].xml" else zin.read(item.filename)
                    zout.writestr(item, data)
    if tmp.exists():
        shutil.move(tmp, DOCX_PATH)
    return DOCX_PATH


def latest_baseline() -> Path | None:
    """The docx of the most recent baseline snapshot, or None."""
    candidates = sorted(BASELINES_DIR.glob("*/" + DOCX_PATH.name))
    return candidates[-1] if candidates else None


def snapshot_baseline() -> Path:
    """Copy the docx, the rendered Markdown and the manifest into Review/baselines/<date>_<n>/."""
    BASELINES_DIR.mkdir(parents=True, exist_ok=True)
    stamp = dt.date.today().isoformat()
    existing = sorted(BASELINES_DIR.glob(f"{stamp}_*"))
    target = BASELINES_DIR / f"{stamp}_{len(existing) + 1:02d}"
    target.mkdir()
    for item in (DOCX_PATH, MARKDOWN_PATH, MANIFEST_PATH):
        shutil.copy2(item, target / item.name)
    return target


def run_gate(baseline: Path | None, intended: list[str]) -> int:
    command = [sys.executable, str(GATE_SCRIPT), str(DOCX_PATH), "--manifest", str(MANIFEST_PATH),
               "--figures-root", str(PROCEDURES_DIR)]
    if baseline is not None:
        command += ["--baseline", str(baseline)]
        if intended:
            command += ["--intended", *intended]
    print("gate:", " ".join(command[1:]))
    return subprocess.run(command, cwd=str(PROCEDURES_DIR)).returncode


# ---------------------------------------------------------------------------
def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--no-docx", action="store_true", help="stop after the Markdown and the manifest")
    parser.add_argument("--no-figures", action="store_true", help="do not regenerate the figures")
    parser.add_argument("--no-gate", action="store_true", help="skip verify_doc.py")
    parser.add_argument("--no-baseline", action="store_true", help="do not archive a dated baseline")
    parser.add_argument("--intended", nargs="*", default=[], help="sections authorized to change (scope diff)")
    args = parser.parse_args(argv)

    params = CharacterizationParameters()
    geometry = SensorGeometry.indicative()
    if not args.no_figures:
        regenerate_figures()
    _, budget = default_plan_and_budget(params, geometry)
    _, totals = budget_table(budget)
    derived = derived_values(params, geometry, totals)
    template = TEMPLATE_PATH.read_text(encoding="utf-8")
    text = render(template, params, geometry, budget, derived)
    MARKDOWN_PATH.write_text(text, encoding="utf-8")
    print(f"wrote {MARKDOWN_PATH}")
    stale = stale_term_hits(text)
    for hit in stale:
        print("STALE:", hit)
    write_manifest(text, derived)
    print(f"wrote {MANIFEST_PATH}")
    if args.no_docx:
        return 1 if stale else 0
    baseline = latest_baseline()
    build_docx()
    print(f"wrote {DOCX_PATH}")
    status = 1 if stale else 0
    if not args.no_gate:
        status = run_gate(baseline, args.intended) or status
    if not args.no_baseline:
        print(f"baseline {snapshot_baseline()}")
    return status


if __name__ == "__main__":
    sys.exit(main())
