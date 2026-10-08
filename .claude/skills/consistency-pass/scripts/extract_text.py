#!/usr/bin/env python3
"""Extract the readable text of the documents a consistency pass compares.

Usage:  python3 extract_text.py <file> [<file> ...] [--out DIR]

For each input file one text file is written, named <basename>.txt, holding every statement
with a location a reader can cite. The files go to --out, or, when --out is not given, to a
fresh folder under the system temporary directory (printed), never next to the inputs: a
document set under review must not acquire stray files.
  .md / .txt   : the file as is, each line prefixed with its line number
  .json        : every string value, prefixed with its key path (slides[3].notes, ...)
  .docx        : the paragraphs and table cells, in document order, numbered
  .pptx        : per slide, the slide's text shapes and tables, then "NOTES:" and the
                 speaker notes; slides numbered from 1
Word and PowerPoint files are read from their XML directly (no third-party packages),
so this runs anywhere Python 3.8 or later runs. Only the standard library is used.
"""
from __future__ import annotations

import json
import re
import sys
import tempfile
import zipfile
from pathlib import Path
from xml.etree import ElementTree as ET

W = "{http://schemas.openxmlformats.org/wordprocessingml/2006/main}"
A = "{http://schemas.openxmlformats.org/drawingml/2006/main}"


def md_text(path: Path) -> str:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(f"{n:5d}: {line}" for n, line in enumerate(lines, 1))


def json_text(path: Path) -> str:
    out: list[str] = []

    def walk(node, key_path: str) -> None:
        if isinstance(node, dict):
            for k, v in node.items():
                walk(v, f"{key_path}.{k}" if key_path else k)
        elif isinstance(node, list):
            for i, v in enumerate(node):
                walk(v, f"{key_path}[{i}]")
        elif isinstance(node, str):
            out.append(f"{key_path}: {node}")
        elif node is not None and not isinstance(node, bool):
            out.append(f"{key_path}: {node}")

    walk(json.loads(path.read_text(encoding="utf-8")), "")
    return "\n".join(out)


def docx_text(path: Path) -> str:
    with zipfile.ZipFile(path) as z:
        root = ET.fromstring(z.read("word/document.xml"))
    out: list[str] = []
    n = 0
    body = root.find(f"{W}body")
    for element in body.iter():
        if element.tag == f"{W}p":
            text = "".join(t.text or "" for t in element.iter(f"{W}t")).strip()
            if text:
                n += 1
                out.append(f"p{n:4d}: {text}")
    return "\n".join(out)


def pptx_text(path: Path) -> str:
    out: list[str] = []
    with zipfile.ZipFile(path) as z:
        names = z.namelist()
        slide_files = sorted(
            (name for name in names if re.fullmatch(r"ppt/slides/slide\d+\.xml", name)),
            key=lambda s: int(re.search(r"(\d+)", s).group(1)))
        for name in slide_files:
            number = int(re.search(r"slide(\d+)", name).group(1))
            out.append(f"=== slide {number}")
            root = ET.fromstring(z.read(name))
            for paragraph in root.iter(f"{A}p"):
                text = "".join(t.text or "" for t in paragraph.iter(f"{A}t")).strip()
                if text:
                    out.append(f"  {text}")
            # Speaker notes: follow the slide's relationship to its notesSlide.
            rel_name = f"ppt/slides/_rels/slide{number}.xml.rels"
            notes_name = None
            if rel_name in names:
                rels = ET.fromstring(z.read(rel_name))
                for rel in rels:
                    if rel.attrib.get("Type", "").endswith("/notesSlide"):
                        notes_name = "ppt/notesSlides/" + Path(rel.attrib["Target"]).name
            if notes_name and notes_name in names:
                notes_root = ET.fromstring(z.read(notes_name))
                notes = [
                    "".join(t.text or "" for t in p.iter(f"{A}t")).strip()
                    for p in notes_root.iter(f"{A}p")]
                notes = [t for t in notes if t and not t.isdigit()]   # drop the slide-number field
                if notes:
                    out.append("  NOTES: " + " ".join(notes))
    return "\n".join(out)


EXTRACTORS = {".md": md_text, ".txt": md_text, ".json": json_text, ".docx": docx_text, ".pptx": pptx_text}


def main(argv: list[str]) -> int:
    out_dir = None
    files: list[Path] = []
    i = 0
    while i < len(argv):
        if argv[i] == "--out":
            out_dir = Path(argv[i + 1]); i += 2
        else:
            files.append(Path(argv[i])); i += 1
    if not files:
        print(__doc__); return 2
    if out_dir is None:
        out_dir = Path(tempfile.mkdtemp(prefix="consistency-pass-"))
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in files:
        extractor = EXTRACTORS.get(f.suffix.lower())
        if extractor is None:
            print(f"skip {f}: no extractor for {f.suffix}"); continue
        text = extractor(f)
        target = out_dir / (f.name + ".txt")
        target.write_text(text, encoding="utf-8")
        print(f"{f} -> {target} ({len(text.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
