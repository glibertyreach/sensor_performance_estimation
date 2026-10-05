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
    {"name": "Noise station count agrees everywhere", "key": "noise_station_count", "value": 11, "phrase": "Z stations"},
    {"name": "Shape station count agrees everywhere", "key": "shape_station_count", "value": 5},
    {"name": "Reduced station count agrees everywhere", "key": "reduced_station_count", "value": 3},
    {"name": "Z-step rung count agrees everywhere", "key": "zstep_rung_count", "value": 6},
    {"name": "Registration pose count agrees everywhere", "key": "registration_poses", "value": 30},
    {"name": "Sentinel frame count agrees everywhere", "key": "sentinel_frames", "value": 30},
    {"name": "Budget total poses agree everywhere", "key": "total_poses", "value": "6,590"},
    {"name": "Budget total frames agree everywhere", "key": "total_frames", "value": "45,710"},
    {"name": "Budget total hours agree everywhere", "key": "total_robot_hours", "value": "6.8"},
    {"name": "Range limits agree everywhere", "key": "z_range_mm", "value": "500–1000"},
    {"name": "Small gap agrees everywhere", "key": "gap_small_mm", "value": 15},
    {"name": "Large gap agrees everywhere", "key": "gap_large_mm", "value": 60},
    {"name": "Extended trials per level agree everywhere", "key": "detection_zero_trials", "value": 300},
    {"name": "Registration acceptance agrees everywhere", "key": "registration_residual_accept_mm", "value": "0.15"},
]
"""The shared numbers the gate proves consistent across sections. Values are the ones the document
holds at the time of declaration; a decided change updates them here AND in the doc."""

VALUE_PATTERNS = {
    "noise_station_count": r"\b11 (?:Z )?stations\b",
    "shape_station_count": r"\b5 (?:shape )?stations\b|Z_SHAPE_STATIONS_MM",
    "reduced_station_count": r"Z_REDUCED_STATIONS_MM",
    "zstep_rung_count": r"Z_STEP_LADDER_MM",
    "registration_poses": r"REGISTRATION_POSES|\b30 poses\b",
    "sentinel_frames": r"\b30 frames\b",
    "total_poses": r"6,590",
    "total_frames": r"45,710",
    "total_robot_hours": r"\b6\.8\b",
    "z_range_mm": r"500–1000|500–1,000",
    "gap_small_mm": r"GAP_SMALL_MM|\b15 mm\b",
    "gap_large_mm": r"GAP_LARGE_MM|\b60 mm\b",
    "detection_zero_trials": r"DETECTION_ZERO_TRIALS|\b300 trials\b",
    "registration_residual_accept_mm": r"REGISTRATION_RESIDUAL_ACCEPT_MM|0\.15 mm",
}
"""Where a section is taken to assert an invariant: the parameter name or the number with its unit."""


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
                if child.findall(".//" + qn("w:drawing")):
                    figure_number += 1
                    elements.append({"id": f"Figure {figure_number}", "kind": "figure", "section": current["id"] if current else "",
                                     "caption": text, "text": ""})
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
        record["source"] = f"Claude Docs {DOC_ID} (live document; edited through the connector)"
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
    p_gate = sub.add_parser("gate"); p_gate.add_argument("--baseline"); p_gate.add_argument("--intended", nargs="*", default=[])
    p_render = sub.add_parser("render"); p_render.add_argument("chunk_id"); p_render.add_argument("--page", default="")
    args = parser.parse_args(argv)
    if args.command == "decode":
        decode(args.export_json)
    elif args.command == "manifest":
        build_manifest()
    elif args.command == "gate":
        return gate(args.baseline, args.intended)
    elif args.command == "render":
        print(render(args.chunk_id, args.page))
    return 0


if __name__ == "__main__":
    sys.exit(main())
