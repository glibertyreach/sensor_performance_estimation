"""
Sidecar of the key values a figure actually drew, so the verification gate
(build/verify_doc.py, check 6) can compare a figure's rendered content with the
document's invariants without reading pixels. Each generator calls
``emit("fig_name", key=value, ...)`` after drawing; the sidecar is written next
to the figure as ``<fig_name>.facts.json``.
(Adapted from the document-review skill's figfacts.py.)
"""
from __future__ import annotations

import json
from pathlib import Path

FIGURES_DIR = Path(__file__).resolve().parent
"""The folder the figures and their sidecars live in."""


def emit(figure_name: str, **facts) -> Path:
    """Write ``<figure_name>.facts.json`` with the facts the figure drew; returns the path."""
    path = FIGURES_DIR / f"{figure_name}.facts.json"
    path.write_text(json.dumps(facts, indent=0, sort_keys=True), encoding="utf-8")
    return path
