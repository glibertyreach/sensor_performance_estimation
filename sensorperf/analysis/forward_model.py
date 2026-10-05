"""
Assembly of ``forward_model_parameters.json`` (document, Section 1 and Section
10 Step 13, Section 14 Step 8): the hand-off from the analyses to the Tier-A
forward-noise simulator.

The mapping the document states:
    sigma_d  (fitted disparity noise, px)        -> sigma_j
    q        (disparity quantum, px)             -> q
    k = f_x B (mm·px)                             -> k
    correlation length (px)                      -> the IDW neighborhood N
    W_fab, W_drop, pi_near (Analysis E)          -> boundary terms

Every analysis result object offers ``forward_model_terms()`` returning the
subset of these it measured (an empty dict when it measured none); this module
merges them, records their provenance, and writes the file. Keys are the
document's symbols in ASCII; units are in the key names.
"""
from __future__ import annotations

import datetime as dt
from pathlib import Path
from typing import Any, Mapping

from sensorperf.analysis.common import write_json
from sensorperf.io.session import FORWARD_MODEL_FILE_NAME

FORMAT_NAME = "sensorperf-forward-model"
FORMAT_VERSION = 1
"""Identity of the JSON format for the consumer."""

TIER_A_MAPPING = {
    "sigma_d_px": "sigma_j",
    "q_px": "q",
    "k_mm_px": "k",
    "corr_len_h_px": "N (IDW neighborhood, along H)",
    "corr_len_v_px": "N (IDW neighborhood, along V)",
    "w_fab_px": "boundary fabrication width",
    "w_drop_px": "boundary dropout width",
    "pi_near": "near/far preference",
    "beta_read": "read versus no-read bias index",
}
"""Our key -> the Tier-A simulator parameter it maps onto (document, Section 1)."""


def assemble(results: Mapping[str, Any], session_root: Path | None = None, sensor_config_id: str = "") -> dict:
    """Merge the forward-model terms of the analysis results (letter -> result).
    A result without ``forward_model_terms`` contributes nothing."""
    terms: dict[str, Any] = {}
    provenance: dict[str, str] = {}
    for letter, result in results.items():
        getter = getattr(result, "forward_model_terms", None)
        if getter is None:
            continue
        for key, value in getter().items():
            terms[key] = value
            provenance[key] = f"analysis {letter}"
    return {
        "format": FORMAT_NAME,
        "version": FORMAT_VERSION,
        "generated": dt.datetime.now().isoformat(timespec="seconds"),
        "session": None if session_root is None else str(session_root),
        "sensor_config_id": sensor_config_id,
        "terms": terms,
        "provenance": provenance,
        "tier_a_mapping": {k: v for k, v in TIER_A_MAPPING.items() if k in terms or k.split("_")[0] in terms},
    }


def write_forward_model(results: Mapping[str, Any], out_dir: Path, session_root: Path | None = None,
                        sensor_config_id: str = "") -> Path:
    """Write forward_model_parameters.json into out_dir and return its path."""
    document = assemble(results, session_root, sensor_config_id)
    return write_json(Path(out_dir) / FORWARD_MODEL_FILE_NAME, document)
