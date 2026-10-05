# Copied from the document-review skill (verify_doc.py) so the repository's build gate is self-contained;
# keep in step with the skill's copy.
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_doc.py — the automated verification gate for the document-review skill.

Given a built .docx (and, optionally, an interrelatedness manifest and a baseline snapshot),
this prints a PASS/FAIL report covering the regression classes that otherwise force the
reader to re-read the whole document:

    1. cross-references resolve          (every section/figure/table mention points to a real element)
    2. inventory is closed               (every figure defined is referenced; live source exists)
    3. term / number consistency         (shared facts agree across every element that asserts them)
    4. figure index complete             (the Figure-Index appendix lists every figure, and only real ones)
    5. prose / index number drift        (a counted noun in prose or an index label agrees with its invariant)
    6. scope diff vs a baseline snapshot (exactly the sections/figures we intended changed)

It reads only the .docx (via python-docx) plus a small JSON manifest, so it needs no build
pipeline.  Usage:

    python verify_doc.py DOC.docx
    python verify_doc.py DOC.docx --manifest Review/manifest.json
    python verify_doc.py DOC.docx --manifest Review/manifest.json --baseline archive/DOC.prev.docx \
                         --intended 13.1 13.2 "Figure Index" --figures-root /path/to/canonical/folder

Exit status is 0 when every applicable check passes, 1 otherwise, so it can gate a build.

CHANGES vs the prior version (2026-07-19 upgrade):
  * NEW check 4 — figure-index completeness (would have caught Figs 24/25 missing from Appendix B).
  * NEW check 5 — prose/index number drift, driven by an optional "phrase" on an invariant
    (would have caught "seven fanless HEPA modules" when hepa_modules == 6).
  * --intended now matches FUZZILY (section number, appendix letter, or normalized substring),
    so `--intended "Figure Index"` matches the section key "Appendix B.  Figure Index".
  * figure sources resolve against a LIST of candidate roots (--figures-root, the docx folder,
    the manifest folder, the cwd), so gating a build from a scratch folder — e.g. when the
    canonical file is locked open — no longer throws false "source missing" failures.
"""
import argparse, json, os, re, sys, hashlib
from docx import Document
from docx.oxml.ns import qn
from docx.table import Table as _Table
from docx.text.paragraph import Paragraph as _Paragraph


def iter_block_items(doc):
    """Yield each Paragraph and Table in DOCUMENT ORDER.  python-docx exposes paragraphs and
    tables in separate collections, which loses their interleaving; walking the body element
    restores it, so table content can be attributed to the section it sits in and scanned for
    references.  (Without this, table cells are invisible to the checks — a real blind spot.)"""
    for child in doc.element.body.iterchildren():
        if child.tag == qn("w:p"):
            yield _Paragraph(child, doc)
        elif child.tag == qn("w:tbl"):
            yield _Table(child, doc)

# --------------------------------------------------------------------------------------------
# Tunable parameters — exposed here rather than hard-coded inline, so the conventions can be
# adjusted per project without editing the logic below.
# --------------------------------------------------------------------------------------------
HEADING_STYLE_PREFIX   = "Heading"          # paragraph style whose text begins with a number
CAPTION_FIGURE_PREFIX  = "Figure"           # figure-caption lead word, e.g. "Figure 19c."
CAPTION_TABLE_PREFIX   = "Table"            # table-caption lead word, e.g. "Table 3."
SECTION_REF_PATTERN    = r"§\s*(\d+(?:\.\d+)*)"        # matches "§7.2.2"
FIGURE_REF_PATTERN     = r"\bFigure\s+(\d+[a-z]?)\b"        # matches "Figure 19c"
TABLE_REF_PATTERN      = r"\bTable\s+(\d+[a-z]?)\b"         # matches "Table 3"
CAPTION_ID_PATTERN     = r"^(?:Figure|Table)\s+(\d+[a-z]?)\b"  # id at the start of a caption line
FIGURE_ID_CELL_PATTERN = r"^\d+[a-z]?$"     # a Figure-Index row's first cell, e.g. "19c"
FIGURE_INDEX_HEADING   = "figure index"     # lower-cased marker for the index appendix heading
PROSE_NUMBER_GAP_WORDS = 3                   # words allowed between a number and its counted noun

# Number-words the prose/index number-drift check understands (0–20 covers counts of parts).
NUMBER_WORDS = {w: i for i, w in enumerate(
    "zero one two three four five six seven eight nine ten eleven twelve thirteen fourteen "
    "fifteen sixteen seventeen eighteen nineteen twenty".split())}

# A "§N" is an EXTERNAL citation (a building code, a standard, or a companion document), not an
# internal cross-reference, when one of these tokens appears shortly before it.  External
# citations are not checked against this document's own headings.
EXTERNAL_REF_KEYWORDS  = ("IBC", "IFC", "NFPA", "IDAPA", "ANSI", "ASHRAE", "AIHA", "OSHA",
                          "RCRA", "ASCE", "IRC", "IECC", "NEC", "Quality_Control", "Quality Control",
                          "QC ", "companion", "architecture specification")
EXTERNAL_KEYWORD_WINDOW = 60      # characters before the "§" to scan for an external keyword


# --------------------------------------------------------------------------------------------
# Extraction — turn the .docx into the small set of facts the checks need.
# --------------------------------------------------------------------------------------------
def load_elements(doc):
    """Return (headings, figures, tables, section_hashes)."""
    headings, figures, tables = [], [], []
    section_hashes, current_key, buffer = {}, None, []

    def flush():
        if current_key is not None:
            digest = hashlib.sha1("\n".join(buffer).encode("utf-8")).hexdigest()
            section_hashes[current_key] = digest

    for block in iter_block_items(doc):
        if isinstance(block, _Table):
            for row in block.rows:                 # fold table cell text into the section hash
                for c in row.cells:
                    buffer.append(c.text)
            continue
        para = block
        style = para.style.name if para.style else ""
        text  = para.text.strip()
        if style.startswith(HEADING_STYLE_PREFIX):
            m = re.match(r"^(\d+(?:\.\d+)*)", text)      # leading "7.2.2"-style number
            flush()
            current_key = m.group(1) if m else text[:40]
            headings.append(current_key)
            buffer = [text]
            continue
        cap = re.match(CAPTION_ID_PATTERN, text)
        if cap:
            (figures if text.startswith(CAPTION_FIGURE_PREFIX) else tables).append(cap.group(1))
        buffer.append(text)
    flush()
    return headings, figures, tables, section_hashes


def load_figure_index(doc):
    """Locate the 'Figure Index' appendix table and return [(id, label), ...] or None.

    Detection: after a heading whose text contains 'figure index', take the first table whose
    first column holds figure ids (e.g. '19c').  A following Heading-1 ends the index region."""
    in_index = False
    for block in iter_block_items(doc):
        if isinstance(block, _Paragraph):
            style = block.style.name if block.style else ""
            t = block.text.strip().lower()
            if style.startswith(HEADING_STYLE_PREFIX) and FIGURE_INDEX_HEADING in t:
                in_index = True
            elif style.startswith("Heading 1") and in_index and FIGURE_INDEX_HEADING not in t:
                break
        elif isinstance(block, _Table) and in_index:
            rows = []
            for r in block.rows:
                cells = [c.text.strip() for c in r.cells]
                if len(cells) >= 2 and re.match(FIGURE_ID_CELL_PATTERN, cells[0]):
                    rows.append((cells[0], cells[1]))
            if rows:
                return rows
    return None


def collect_body_refs(doc, max_toplevel):
    """Return the sets of section / figure / table numbers *mentioned* in the body text
    (excluding caption lines, which define rather than reference)."""
    sect, fig, tab = set(), set(), set()

    def scan(text):
        for m in re.finditer(SECTION_REF_PATTERN, text):
            num = m.group(1); top = int(num.split(".")[0])
            if top > max_toplevel:                        # e.g. §707, §714 — a code citation
                continue
            window = text[max(0, m.start() - EXTERNAL_KEYWORD_WINDOW): m.start()]
            if any(k in window for k in EXTERNAL_REF_KEYWORDS):   # e.g. companion QC doc ref
                continue
            sect.add(num)
        fig.update(re.findall(FIGURE_REF_PATTERN, text))
        tab.update(re.findall(TABLE_REF_PATTERN, text))

    for block in iter_block_items(doc):
        if isinstance(block, _Table):
            for row in block.rows:
                for c in row.cells:
                    scan(c.text)
            continue
        if re.match(CAPTION_ID_PATTERN, block.text.strip()):   # skip caption definitions
            continue
        scan(block.text)
    return sect, fig, tab


def collect_all_text(doc):
    """Every paragraph and table-cell string in the document (used by the prose-number check)."""
    out = []
    for block in iter_block_items(doc):
        if isinstance(block, _Table):
            for row in block.rows:
                for c in row.cells:
                    out.append(c.text)
        else:
            out.append(block.text)
    return out


# --------------------------------------------------------------------------------------------
# The checks.  Each returns (ok: bool, list_of_messages).
# --------------------------------------------------------------------------------------------
def check_cross_references(headings, figures, tables, ref_sect, ref_fig, ref_tab):
    msgs, ok = [], True
    heading_set = set(headings)
    for s in sorted(ref_sect):
        if s not in heading_set and not any(h == s or h.startswith(s + ".") for h in heading_set):
            ok = False; msgs.append(f"  · §{s} is referenced but no such heading exists")
    for f in sorted(ref_fig):
        if f not in set(figures):
            ok = False; msgs.append(f"  · Figure {f} is referenced but has no caption")
    for t in sorted(ref_tab):
        if t not in set(tables):
            ok = False; msgs.append(f"  · Table {t} is referenced but has no caption")
    return ok, msgs


def check_inventory(figures, ref_fig, manifest, roots):
    """roots: ordered list of candidate directories a relative figure-source path may live under.
    A source that resolves under ANY of them passes; only one that resolves under NONE fails."""
    msgs, ok = [], True
    defined = set(figures)
    dupes = {f for f in figures if figures.count(f) > 1}
    if dupes:
        ok = False; msgs.append(f"  · duplicate figure captions: {sorted(dupes)}")
    for f in sorted(ref_fig - defined):
        ok = False; msgs.append(f"  · Figure {f} is referenced but never captioned/embedded")
    for f in sorted(defined - ref_fig):
        msgs.append(f"  · (warn) Figure {f} is captioned but never referenced in the text")
    if manifest:
        by_id = {e["id"]: e for e in manifest.get("elements", [])}
        for f in sorted(defined):
            fid = f"Figure {f}"
            src = by_id.get(fid, {}).get("source")
            if not src:
                msgs.append(f"  · (warn) {fid} not yet in the manifest — add its source when reviewed")
            elif src not in ("inline", "python-docx"):
                if os.path.isabs(src):
                    found = os.path.exists(src)
                else:
                    found = any(os.path.exists(os.path.join(r, src)) for r in roots if r)
                if not found:
                    ok = False
                    msgs.append(f"  · {fid} source script is RECORDED BUT MISSING under any root: {src}")
    return ok, msgs


def _invariant_value(manifest, key):
    """The single agreed value of an invariant key across the elements that assert it, or None."""
    vals = {e["asserts"][key] for e in manifest.get("elements", []) if key in e.get("asserts", {})}
    return next(iter(vals)) if len(vals) == 1 else None


def check_consistency(manifest):
    """For each invariant key, every element that asserts that key must hold the same value."""
    if not manifest:
        return True, ["  · (no manifest supplied — term/number consistency not checked)"]
    msgs, ok = [], True
    elements = manifest.get("elements", [])
    for inv in manifest.get("invariants", []):
        key = inv["key"]
        holders = [(e["id"], e["asserts"][key]) for e in elements if key in e.get("asserts", {})]
        values = {v for _, v in holders}
        if len(values) > 1:
            ok = False
            detail = ", ".join(f"{eid}={val}" for eid, val in holders)
            msgs.append(f"  · '{inv['name']}' [{key}] disagrees: {detail}")
    return ok, msgs


def check_figure_index(figures, index_entries):
    """The Figure-Index appendix must list every real figure exactly once, and nothing else.
    This catches the class of defect where a new figure is added but the hand-kept index is
    not updated (Figs 24/25), or the index still lists a figure that was removed/renumbered."""
    if index_entries is None:
        return True, ["  · (no 'Figure Index' table found — index completeness not checked)"]
    ok, msgs = True, []
    idx_ids = [i for i, _ in index_entries]
    idx_set, fig_set = set(idx_ids), set(figures)
    dupes = {i for i in idx_ids if idx_ids.count(i) > 1}
    if dupes:
        ok = False; msgs.append(f"  · Figure Index lists duplicate ids: {sorted(dupes)}")
    for f in sorted(fig_set - idx_set):
        ok = False; msgs.append(f"  · Figure {f} is in the document but MISSING from the Figure Index")
    for f in sorted(idx_set - fig_set):
        ok = False; msgs.append(f"  · Figure Index lists Figure {f}, which the document does not contain")
    return ok, msgs


def check_prose_numbers(all_text, manifest):
    """For any invariant that declares a "phrase" (the counted noun, e.g. "HEPA module"), scan
    the whole document — prose AND table/index labels — for "<number> ... <phrase>" and flag a
    number that disagrees with the invariant's agreed value.  This extends the term/number check
    out of the manifest and into free text, where stale counts (like a figure-index label that
    still says "seven") otherwise hide."""
    if not manifest:
        return True, ["  · (no manifest — prose/index number drift not checked)"]
    phrased = [inv for inv in manifest.get("invariants", []) if inv.get("phrase")]
    if not phrased:
        return True, ["  · (no invariant declares a 'phrase' — prose/index number drift not checked)"]
    ok, msgs = True, []
    number_alt = r"\d+|" + "|".join(NUMBER_WORDS)
    for inv in phrased:
        key = inv["key"]
        val = _invariant_value(manifest, key)
        if val is None:
            continue
        try:
            valn = int(val)
        except (TypeError, ValueError):
            continue
        phrase = re.escape(inv["phrase"]) + r"s?"     # tolerate a trailing plural 's'
        pat = re.compile(r"\b(" + number_alt + r")\b(?:\s+\w+){0," + str(PROSE_NUMBER_GAP_WORDS)
                         + r"}?\s+" + phrase, re.I)
        for t in all_text:
            for m in pat.finditer(t):
                tok = m.group(1).lower()
                n = int(tok) if tok.isdigit() else NUMBER_WORDS.get(tok)
                if n is not None and n != valn:
                    ok = False
                    msgs.append(f"  · '{inv.get('name', key)}' [{key}={valn}] contradicted by "
                                f"\"{m.group(0).strip()}\"")
    return ok, msgs


def check_figure_facts(manifest, roots):
    """Compare each figure's emitted FACTS sidecar (what its generator ACTUALLY drew) to the
    manifest invariants (the canonical value).  This closes the gate's biggest blind spot: the
    numbers baked into a rendered figure are opaque to a text checker, so a figure could show
    "7 HEPA" while the prose and every other figure say 6 and nothing here would notice.  Each
    figure script writes <name>.facts.json via figfacts.emit(); a figure element that names it in
    'facts' is checked here."""
    if not manifest:
        return True, ["  · (no manifest — figure-facts check skipped)"]
    inv_val = {}
    for inv in manifest.get("invariants", []):
        v = _invariant_value(manifest, inv["key"])
        if v is not None:
            inv_val[inv["key"]] = v
    ok, msgs, checked = True, [], 0
    for e in manifest.get("elements", []):
        fp = e.get("facts")
        if not fp:
            continue
        path = (fp if os.path.isabs(fp) else
                next((os.path.join(r, fp) for r in roots if r and os.path.exists(os.path.join(r, fp))), None))
        if not path:
            msgs.append(f"  · (warn) {e['id']} declares facts '{fp}' but the sidecar is missing (regenerate the figure)")
            continue
        try:
            facts = json.load(open(path, encoding="utf-8"))
        except Exception as ex:
            ok = False; msgs.append(f"  · {e['id']} facts sidecar unreadable: {ex}"); continue
        checked += 1
        for k, v in facts.items():
            if k in inv_val and v != inv_val[k]:
                ok = False
                msgs.append(f"  · {e['id']} DREW {k}={v} but the invariant says {k}={inv_val[k]} — figure is stale/divergent")
    if checked == 0:
        msgs.append("  · (no figure declares a 'facts' sidecar yet — wire figfacts.emit() into the generators)")
    return ok, msgs


def _authorized(changed_key, intended):
    """A changed section key is authorised if any --intended token matches it fuzzily: equal,
    a normalized substring either way, or the same leading section number / appendix letter."""
    ck = re.sub(r"\s+", " ", changed_key).strip().lower()
    ck_lead = ck.split()[0].rstrip(".") if ck.split() else ck
    for tok in intended:
        t = re.sub(r"\s+", " ", tok).strip().lower()
        if not t:
            continue
        if t == ck or t in ck or ck in t or t == ck_lead:
            return True
    return False


def check_scope(current_hashes, baseline_docx, intended):
    if not baseline_docx:
        return True, ["  · (no baseline supplied — scope diff not checked)"]
    base = Document(baseline_docx)
    _, _, _, base_hashes = load_elements(base)
    changed = {k for k, v in current_hashes.items() if base_hashes.get(k) != v}
    changed |= (set(base_hashes) - set(current_hashes))     # removed sections
    changed |= (set(current_hashes) - set(base_hashes))     # added sections
    msgs = [f"  · sections changed vs baseline: {sorted(changed) or 'none'}"]
    ok = True
    if intended is not None:
        unexpected = [c for c in sorted(changed) if not _authorized(c, intended)]
        if unexpected:
            ok = False
            msgs.append(f"  · UNEXPECTED changes outside the authorised scope: {unexpected}")
    return ok, msgs


# --------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Verification gate for chunked document review.")
    ap.add_argument("docx")
    ap.add_argument("--manifest", help="Review/manifest.json (elements + invariants)")
    ap.add_argument("--baseline", help="prior .docx snapshot for the scope diff")
    ap.add_argument("--intended", nargs="*", help="section numbers/labels the reviewer authorised to change")
    ap.add_argument("--figures-root", help="extra directory to resolve relative figure-source paths against "
                                           "(use when gating a build whose figures/ live elsewhere, e.g. a "
                                           "locked canonical folder)")
    args = ap.parse_args()

    doc = Document(args.docx)
    manifest = json.load(open(args.manifest, encoding="utf-8")) if args.manifest else None
    # Candidate roots for relative figure-source paths, most-specific first.
    roots = []
    if args.figures_root:
        roots.append(os.path.abspath(args.figures_root))
    roots.append(os.path.dirname(os.path.abspath(args.docx)))          # the docx's own folder
    if args.manifest:
        roots.append(os.path.dirname(os.path.abspath(args.manifest)))  # the manifest's folder
    roots.append(os.getcwd())

    headings, figures, tables, hashes = load_elements(doc)
    index_entries = load_figure_index(doc)
    all_text = collect_all_text(doc)
    max_toplevel = max((int(h.split(".")[0]) for h in headings if h.split(".")[0].isdigit()), default=0)
    ref_sect, ref_fig, ref_tab = collect_body_refs(doc, max_toplevel)

    checks = [
        ("1. cross-references resolve", check_cross_references(headings, figures, tables, ref_sect, ref_fig, ref_tab)),
        ("2. inventory closed",        check_inventory(figures, ref_fig, manifest, roots)),
        ("3. term/number consistency", check_consistency(manifest)),
        ("4. figure index complete",   check_figure_index(figures, index_entries)),
        ("5. prose/index number drift",check_prose_numbers(all_text, manifest)),
        ("6. figure facts vs invariants", check_figure_facts(manifest, roots)),
        ("7. scope diff",              check_scope(hashes, args.baseline, args.intended)),
    ]

    print(f"\nVERIFICATION REPORT — {os.path.basename(args.docx)}")
    print(f"  {len(headings)} headings · {len(figures)} figures · {len(tables)} tables · "
          f"index {'present' if index_entries else 'absent'}\n")
    all_ok = True
    for name, (ok, msgs) in checks:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")
        for m in msgs:
            print(m)
        all_ok = all_ok and ok
    print(f"\n==> {'ALL CHECKS PASSED' if all_ok else 'FAILURES ABOVE — do not ship until resolved'}\n")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
