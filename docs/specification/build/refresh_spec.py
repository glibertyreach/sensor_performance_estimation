#!/usr/bin/env python3
"""
Review tooling for the characterization SPECIFICATION, whose source of truth is the live
Claude Docs document (https://claude.ai/code/artifact/20fd839b-0e95-4210-9e8f-959da07ce6c2).

The loop of the document-review skill, adapted to a Docs-hosted source:
    1. the doc is exported to .docx with the Docs connector (export, format docx); the tool result
       is a JSON file holding the base64 payload;
    2. `refresh_spec.py decode <export-json>` writes VSX3000_characterization_specification.docx
       next to this folder and archives a dated copy under archive/;
    3. `refresh_spec.py manifest` rebuilds Review/manifest.json from the .docx outline and the
       INVARIANTS declared below (the asserted values are found by scanning each section's text);
    4. `refresh_spec.py gate [--baseline archive/<file>] [--intended ...]` runs the verification
       gate (the copy of verify_doc.py in docs/procedures/build);
    5. `refresh_spec.py render <chunk-id> [--page N]` renders one review chunk to Review/chunks/.
Edits are made in the live doc through the connector (never in the .docx); after each edit the
doc is exported again and steps 2 to 4 repeat, so the gate always judges the freshly exported copy.
"""
from __future__ import annotations

import argparse
import base64
import datetime as dt
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SPEC_DIR = HERE.parent
DOCX_PATH = SPEC_DIR / "VSX3000_characterization_specification.docx"
REVIEW_DIR = SPEC_DIR / "Review"
ARCHIVE_DIR = SPEC_DIR / "archive"
CHUNKS_DIR = REVIEW_DIR / "chunks"
MANIFEST_PATH = REVIEW_DIR / "manifest.json"
PROGRESS_PATH = REVIEW_DIR / "progress.json"
CHUNK_PLAN_PATH = REVIEW_DIR / "chunk_plan.json"
GATE_SCRIPT = SPEC_DIR.parent / "procedures" / "build" / "verify_doc.py"
RENDER_SCRIPT = HERE / "render_subsection.py"
DOC_ID = "20fd839b-0e95-4210-9e8f-959da07ce6c2"
RUNNING_HEADER = "VSX3000 characterization specification | review"

INVARIANTS = [
    {"name": "Station ladder count agrees everywhere", "key": "noise_station_count", "value": 9, "phrase": "Z stations"},
    {"name": "Shape station count agrees everywhere", "key": "shape_station_count", "value": 5},
    {"name": "Reduced station count agrees everywhere", "key": "reduced_station_count", "value": 3},
    {"name": "Z-step rung count agrees everywhere", "key": "zstep_rung_count", "value": 6},
    {"name": "Registration pose count agrees everywhere", "key": "registration_poses", "value": 30},
    {"name": "Sentinel frame count agrees everywhere", "key": "sentinel_frames", "value": 30},
    {"name": "Budget total poses agree everywhere", "key": "total_poses", "value": "7,194"},
    {"name": "Budget total frames agree everywhere", "key": "total_frames", "value": "42,520"},
    {"name": "Budget total hours agree everywhere", "key": "total_robot_hours", "value": "7.18"},
    {"name": "Range limits agree everywhere", "key": "z_range_mm", "value": "400–1600"},
    {"name": "Feature ladder agrees everywhere", "key": "feature_diameters_mm", "value": "7.0, 19.7, 55.8"},
    {"name": "Station ladder values agree everywhere", "key": "station_ladder_mm", "value": "400, 476, 566, 673, 800, 951, 1131, 1345, 1600"},
    {"name": "Features per plate agree everywhere", "key": "feature_count", "value": 3},
    {"name": "Small gap agrees everywhere", "key": "gap_small_mm", "value": 15},
    {"name": "Large gap agrees everywhere", "key": "gap_large_mm", "value": 60},
    {"name": "Extended low-point trials agree everywhere", "key": "detection_zero_trials", "value": 300},
    {"name": "Registration acceptance agrees everywhere", "key": "registration_residual_accept_mm", "value": "0.15"},
]
"""The shared numbers the gate proves consistent across sections. Values are the ones the document
holds at the time of declaration; a decided change updates them here AND in the doc."""

VALUE_PATTERNS = {
    "noise_station_count": r"\b9 (?:Z )?stations\b|Z_STATION_RATIO",
    "shape_station_count": r"\b5 (?:shape )?stations\b|Z_SHAPE_STATION_STRIDE",
    "reduced_station_count": r"Z_REDUCED_STATION_STRIDE",
    "zstep_rung_count": r"Z_STEP_LADDER_QUANTA|\b6 rungs\b",
    "registration_poses": r"REGISTRATION_POSES|\b30 poses\b",
    "sentinel_frames": r"\b30 frames\b",
    "total_poses": r"7,194",
    "total_frames": r"42,520",
    "total_robot_hours": r"\b7\.18\b",
    "z_range_mm": r"400–1600|400–1,600|400 and 1600 mm",
    "feature_diameters_mm": r"7\.0, 19\.7,? (?:and )?55\.8",
    "station_ladder_mm": r"400, 476, 566, 673, 800, 951, 1131, 1345, 1600",
    "feature_count": r"FEATURE_COUNT",
    "gap_small_mm": r"GAP_SMALL_MM|\b15 mm\b",
    "gap_large_mm": r"GAP_LARGE_MM|\b60 mm\b",
    "detection_zero_trials": r"DETECTION_LOW_TRIALS|\b300 trials\b",
    "registration_residual_accept_mm": r"REGISTRATION_RESIDUAL_ACCEPT_MM|0\.15 mm",
}
"""Where a section is taken to assert an invariant: the parameter name or the number with its unit."""


# ---------------------------------------------------------------------------
# Formula typesetting for the review PDF. The live document keeps each display
# formula as a ```latex block, which the Docs viewer renders as math; the .docx
# export carries only the bare TeX in a "Code" paragraph. For the review render
# the TeX is typeset with matplotlib's mathtext and placed as an inline picture,
# with the TeX kept as the picture's alt text.
# ---------------------------------------------------------------------------
FIGURE_SOURCES = ["figures/figure1_procedure_flow.jsx", "figures/figure2_chamfer_section.jsx",
                  "figures/figure3_setup_side_view.jsx"]
"""Exported copies of the three figure widgets' modules, in document order. The live widget in
the Claude Docs document is the source of truth; refresh the copy whenever the widget is republished."""
LIVE_SOURCE = "Claude Docs 20fd839b-0e95-4210-9e8f-959da07ce6c2 (live document; edited through the connector)"
FORMULA_STYLE_NAME = "Code"
"""Paragraph style the export gives a ```latex block."""
FORMULA_FONT_PT = 11.0
"""Type size of the typeset formula, matching the body text."""
FORMULA_DPI = 300
"""Rendering resolution of the formula pictures."""
FORMULA_MAX_WIDTH_IN = 6.2
"""Widest picture that fits the page's text column; wider renders are scaled down."""
FORMULA_DIR_NAME = "formulas"
TEX_COMMAND_PATTERN = r"\\[A-Za-z]+"
"""A backslash command marks a picture's alt text as TeX, so the picture is a formula, not a figure."""
"""Sub-directory of archive/ that holds the rendered formula pictures."""
TEX_TO_MATHTEXT = [
    (r"\qquad", r"\quad\quad"), (r"\big(", "("), (r"\big)", ")"), (r"\Big(", "("), (r"\Big)", ")"),
    (r"\!", ""), (r"\;", r"\,"), (r"\mid", "|"),
]
"""Literal TeX constructs mathtext does not accept, with their mathtext spelling."""


def tex_to_mathtext(tex: str) -> str:
    """Rewrite bare display TeX into the subset matplotlib's mathtext parses."""
    text = tex.strip()
    for old, new in TEX_TO_MATHTEXT:
        text = text.replace(old, new)
    text = re.sub(r"\\text\{([^}]*)\}", lambda m: r"\mathrm{" + m.group(1) + "}", text)
    text = re.sub(r"\\le\b", r"\\leq", text)
    text = re.sub(r"\\ge\b", r"\\geq", text)
    return text


def render_formula(tex: str, out_png: Path) -> tuple[float, float]:
    """Typeset one formula to a PNG; returns its (width, height) in inches at FORMULA_DPI."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["mathtext.fontset"] = "cm"
    figure = plt.figure(figsize=(0.1, 0.1))
    figure.text(0, 0, "$" + tex_to_mathtext(tex) + "$", fontsize=FORMULA_FONT_PT)
    figure.savefig(out_png, dpi=FORMULA_DPI, bbox_inches="tight", pad_inches=0.04, transparent=False)
    plt.close(figure)
    from PIL import Image
    with Image.open(out_png) as image:
        width_px, height_px = image.size
    return width_px / FORMULA_DPI, height_px / FORMULA_DPI


def typeset_formulas(docx_path: Path = DOCX_PATH) -> int:
    """Replace every TeX 'Code' paragraph of the .docx with its typeset picture; returns the count."""
    from docx import Document
    from docx.shared import Inches
    doc = Document(str(docx_path))
    out_dir = ARCHIVE_DIR / FORMULA_DIR_NAME
    out_dir.mkdir(parents=True, exist_ok=True)
    count = 0
    for paragraph in doc.paragraphs:
        if paragraph.style.name != FORMULA_STYLE_NAME or "\\" not in paragraph.text:
            continue
        tex = paragraph.text.strip()
        count += 1
        png = out_dir / f"formula_{count:02d}.png"
        width_in, height_in = render_formula(tex, png)
        scale = min(1.0, FORMULA_MAX_WIDTH_IN / width_in)
        for run in list(paragraph.runs):
            run._element.getparent().remove(run._element)
        paragraph.style = doc.styles["Normal"]
        picture = paragraph.add_run().add_picture(str(png), width=Inches(width_in * scale),
                                                  height=Inches(height_in * scale))
        picture._inline.docPr.set("descr", tex)
    doc.save(str(docx_path))
    print(f"typeset {count} formulas into {docx_path.name} (pictures under {out_dir})")
    return count


def decode(export_json: Path) -> Path:
    raw = export_json.read_text(encoding="utf-8")
    try:
        document = json.loads(raw)
    except json.JSONDecodeError:
        document = None
    payload = None
    if document is not None:
        stack = [document]
        while stack and payload is None:
            item = stack.pop()
            if isinstance(item, dict):
                stack.extend(item.values())
            elif isinstance(item, list):
                stack.extend(item)
            elif isinstance(item, str) and len(item) > 50000:
                payload = item
    if payload is None:
        match = re.search(r"[A-Za-z0-9+/=]{50000,}", raw)
        payload = match.group(0) if match else None
    if payload is None:
        raise SystemExit("no base64 payload found in the export result")
    DOCX_PATH.write_bytes(base64.b64decode(payload))
    ARCHIVE_DIR.mkdir(exist_ok=True)
    typeset_formulas(DOCX_PATH)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    archived = ARCHIVE_DIR / f"{DOCX_PATH.stem}_{stamp}.docx"
    shutil.copy2(DOCX_PATH, archived)
    print(f"wrote {DOCX_PATH} ({DOCX_PATH.stat().st_size} bytes); archived {archived}")
    return archived


def outline():
    """(kind, id, title, text) per section, figure caption and table of the .docx, in order."""
    from docx import Document
    from docx.oxml.ns import qn
    from docx.table import Table
    from docx.text.paragraph import Paragraph
    doc = Document(str(DOCX_PATH))
    elements = []
    current = None
    figure_number = 0
    table_number = 0
    for child in doc.element.body.iterchildren():
        if child.tag == qn("w:p"):
            paragraph = Paragraph(child, doc)
            style = paragraph.style.name if paragraph.style is not None else ""
            text = paragraph.text.strip()
            if style.startswith("Heading") and text:
                number = re.match(r"^(\d+(?:\.\d+)*)", text)
                key = f"§{number.group(1)}" if number else text
                current = {"id": key, "kind": "section", "title": text, "text": ""}
                elements.append(current)
            else:
                if current is not None:
                    current["text"] += text + "\n"
                drawings = child.findall(".//" + qn("w:drawing"))
                # A typeset formula is a picture whose alt text is its TeX; it is not a figure.
                is_formula = any(re.search(TEX_COMMAND_PATTERN, (d.find(".//" + qn("wp:docPr")).get("descr") or ""))
                                 for d in drawings if d.find(".//" + qn("wp:docPr")) is not None)
                if drawings and not is_formula:
                    figure_number += 1
                    figure_source = (FIGURE_SOURCES[figure_number - 1] if figure_number <= len(FIGURE_SOURCES)
                                     else LIVE_SOURCE)
                    elements.append({"id": f"Figure {figure_number}", "kind": "figure", "section": current["id"] if current else "",
                                     "caption": text, "text": "", "source": figure_source})
        elif child.tag == qn("w:tbl"):
            table_number += 1
            table = Table(child, doc)
            cells = "\n".join(" | ".join(c.text for c in row.cells) for row in table.rows)
            if current is not None:
                current["text"] += cells + "\n"
            elements.append({"id": f"Table {table_number}", "kind": "table", "section": current["id"] if current else "",
                             "caption": table.rows[0].cells[0].text if table.rows else "", "text": cells})
    return elements


def build_manifest() -> dict:
    elements = []
    for element in outline():
        asserts = {}
        for invariant in INVARIANTS:
            pattern = VALUE_PATTERNS[invariant["key"]]
            if re.search(pattern, element["text"]):
                asserts[invariant["key"]] = invariant["value"]
        record = {k: v for k, v in element.items() if k != "text"}
        if record.get("kind") != "figure":
            record["source"] = LIVE_SOURCE
        record["asserts"] = asserts
        record["refs"] = sorted(set(re.findall(r"Section \d+(?:\.\d+)?|Step \d+\.\d+", element["text"])))
        elements.append(record)
    manifest = {"document": DOCX_PATH.name, "built": dt.datetime.now().isoformat(timespec="seconds"),
                "source_of_truth": f"https://claude.ai/code/artifact/{DOC_ID}",
                "elements": elements,
                "invariants": [{k: v for k, v in i.items() if k != "value"} for i in INVARIANTS],
                "invariant_values": {i["key"]: i["value"] for i in INVARIANTS}}
    REVIEW_DIR.mkdir(exist_ok=True)
    MANIFEST_PATH.write_text(json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8")
    sections = [e for e in elements if e["kind"] == "section"]
    figures = [e for e in elements if e["kind"] == "figure"]
    tables = [e for e in elements if e["kind"] == "table"]
    print(f"manifest: {len(sections)} sections, {len(figures)} figures, {len(tables)} tables")
    return manifest


def gate(baseline: str | None, intended: list[str]) -> int:
    command = [sys.executable, str(GATE_SCRIPT), str(DOCX_PATH), "--manifest", str(MANIFEST_PATH),
               "--figures-root", str(SPEC_DIR)]
    if baseline:
        command += ["--baseline", baseline]
        if intended:
            command += ["--intended", *intended]
    return subprocess.run(command, cwd=str(SPEC_DIR)).returncode


def render(chunk_id: str, page: str) -> Path:
    plan = json.loads(CHUNK_PLAN_PATH.read_text(encoding="utf-8"))
    chunk = next((c for c in plan["chunks"] if c["id"] == chunk_id), None)
    if chunk is None:
        raise SystemExit(f"unknown chunk {chunk_id}; see {CHUNK_PLAN_PATH}")
    CHUNKS_DIR.mkdir(parents=True, exist_ok=True)
    outputs = []
    for heading in chunk["headings"]:
        safe = re.sub(r"[^A-Za-z0-9]+", "_", heading).strip("_")
        out = CHUNKS_DIR / f"{chunk_id}_{safe}.pdf"
        subprocess.run([sys.executable, str(RENDER_SCRIPT), str(DOCX_PATH), heading, str(out),
                        "--header", RUNNING_HEADER, "--page", page], check=True)
        outputs.append(out)
    if len(outputs) == 1:
        return outputs[0]
    # Several headings in one chunk: merge the pages into one PDF.
    from pypdf import PdfWriter
    writer = PdfWriter()
    for part in outputs:
        writer.append(str(part))
    merged = CHUNKS_DIR / f"{chunk_id}.pdf"
    with merged.open("wb") as handle:
        writer.write(handle)
    return merged


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    p_decode = sub.add_parser("decode"); p_decode.add_argument("export_json", type=Path)
    sub.add_parser("manifest")
    sub.add_parser("typeset")
    p_gate = sub.add_parser("gate"); p_gate.add_argument("--baseline"); p_gate.add_argument("--intended", nargs="*", default=[])
    p_render = sub.add_parser("render"); p_render.add_argument("chunk_id"); p_render.add_argument("--page", default="")
    args = parser.parse_args(argv)
    if args.command == "decode":
        decode(args.export_json)
    elif args.command == "typeset":
        typeset_formulas()
    elif args.command == "manifest":
        build_manifest()
    elif args.command == "gate":
        return gate(args.baseline, args.intended)
    elif args.command == "render":
        print(render(args.chunk_id, args.page))
    return 0


if __name__ == "__main__":
    sys.exit(main())
