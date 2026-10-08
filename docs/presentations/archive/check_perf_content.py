"""Check both VSX3000 performance-testing decks against their content JSONs: every non-notes text string in the
JSON appears on its slide in the .pptx (via markitdown), and every slide's speaker notes equal the JSON notes.
Then check the build deck's cost figures against docs/procedures/build/costs.py, their single source (read
through `costs.py --json`): the cost slide's table, stats and title, the build_list and buy_list rows, and every
dollar range anywhere in either deck's text or notes.

Run from anywhere:  python3 docs/presentations/archive/check_perf_content.py   (needs markitdown[pptx] and python-pptx)"""
import json, re, subprocess, sys
from pathlib import Path
from pptx import Presentation

REPO = Path(__file__).resolve().parents[3]  # archive/ -> presentations/ -> docs/ -> repository root
DECKS = [  # (content JSON, built deck)
    (REPO / "docs/presentations/build/perf_build_deck_content.json", REPO / "docs/presentations/vsx3000_procurement_build.pptx"),
    (REPO / "docs/presentations/build/perf_procedure_deck_content.json", REPO / "docs/presentations/vsx3000_test_procedure.pptx"),
]
COSTS_PY = REPO / "docs/procedures/build/costs.py"
SKIP_KEYS = {"id", "builder", "layout", "section", "notes", "icon", "image", "images", "n"}  # not slide text

def norm(t):
    t = t.replace(" ", " ").replace("\\", "")      # non-breaking space, markdown escapes
    t = re.sub(r"\s+", " ", t)
    return t.strip()

def strings(obj):
    if isinstance(obj, str):
        yield obj
    elif isinstance(obj, list):
        for v in obj: yield from strings(v)
    elif isinstance(obj, dict):
        for k, v in obj.items():
            if k not in SKIP_KEYS: yield from strings(v)

def check(content_json, pptx):
    """Return the number of problems found in one deck."""
    md = subprocess.run(["markitdown", str(pptx)], capture_output=True, text=True, check=True).stdout
    parts = re.split(r"<!-- Slide number: (\d+) -->", md)
    # markitdown appends "### Notes:" to each slide block; keep the slide body and the notes apart
    slide_text, notes_text = {}, {}
    for i in range(1, len(parts), 2):
        body, _, notes = parts[i + 1].partition("### Notes:")
        slide_text[int(parts[i])], notes_text[int(parts[i])] = norm(body), norm(notes)
    slides = json.load(open(content_json))["slides"]
    prs = Presentation(str(pptx))
    bad = 0
    assert len(slides) == len(prs.slides) == len(slide_text), (len(slides), len(prs.slides), len(slide_text))
    for i, s in enumerate(slides, 1):
        text = slide_text[i]
        for t in strings(s):
            if norm(t) not in text:
                bad += 1; print(f"slide {i} ({s['id']}): MISSING {t!r}")
        notes = prs.slides[i - 1].notes_slide.notes_text_frame.text if prs.slides[i - 1].has_notes_slide else ""
        if not notes.strip() or norm(notes) != norm(s["notes"]):
            bad += 1; print(f"slide {i} ({s['id']}): NOTES differ or empty")
        # notes text must not be on the slide itself
        if norm(s["notes"]) != notes_text[i]:
            bad += 1; print(f"slide {i}: markitdown notes differ from JSON notes")
        if norm(s["notes"])[:60] in text:
            bad += 1; print(f"slide {i}: notes text found on the slide body")
    print(f"{pptx.name}: checked {len(slides)} slides; problems: {bad}")
    return bad


# ---- cost consistency: the build deck's figures against costs.py --------------------------------------
COST_HEADER = "Estimated cost (USD)"
RANGE = re.compile(r"\$[\d,]+ to \$[\d,]+")


def load_costs():
    return json.loads(subprocess.run([sys.executable, str(COSTS_PY), "--json"], capture_output=True, text=True, check=True).stdout)


def usd_range(low, high):
    """costs.py's own format (usd_range), repeated here so that the check reads only the JSON."""
    return "not estimated" if (low, high) == (0, 0) else f"${low:,} to ${high:,}"


def check_costs(content_json):
    """Compare every cost string of the build deck with costs.py --json; return the problems found."""
    j = load_costs()
    slides = {s["id"]: s for s in json.load(open(content_json))["slides"]}
    items = j["build"] + j["buy"]
    bad = 0

    def report(label, ok, detail=""):
        nonlocal bad
        bad += 0 if ok else 1
        print(f"  {label}{detail}: {'ok' if ok else 'MISMATCH'}")

    # build_list: every costs.py build item once, matched by its drawing number; name, quantity, drawing and source agree
    table = slides["build_list"]["table"]
    col = {h: n for n, h in enumerate(table["header"])}
    by_drawing = {it["drawing"]: it for it in j["build"]}
    seen = set()
    for row in table["rows"]:
        it = by_drawing.get(row[col["Drawing"]])
        if it is None:
            report(f"build_list {row[0]!r}", False, " (no costs.py item with this drawing)")
            continue
        seen.add(it["drawing"])
        ok = row[0].startswith(it["name"]) and row[col["Quantity"]] == it["quantity"] and row[col["From"]] == (it["from_project"] or "new")
        report(f"build_list {row[0]!r} ~ {it['name']!r}, quantity {row[col['Quantity']]!r}, from {row[col['From']]!r}", ok)
    report("build_list covers every costs.py build item", seen == set(by_drawing), f" (missing: {sorted(set(by_drawing) - seen)})")

    # cost: the new build items, matched by the drawing number in parentheses; the cost range and nothing else of them
    table = slides["cost"]["table"]
    ci = table["header"].index(COST_HEADER)
    seen = set()
    for row in table["rows"]:
        m = re.search(r"\((PT-\d+|SC1-\d+)\)", row[0])
        it = by_drawing.get(m.group(1)) if m else None
        if it is None:
            report(f"cost {row[0]!r}", False, " (no costs.py item for this row)")
            continue
        seen.add(it["drawing"])
        want = usd_range(it["low"], it["high"])
        report(f"cost {row[0]!r} ~ {it['name']!r}: deck {row[ci]!r} vs costs.py {want!r}", row[ci] == want)
    new_drawings = {d for d, it in by_drawing.items() if not it["from_project"]}
    report("cost covers every new build item and no other", seen == new_drawings, f" (missing: {sorted(new_drawings - seen)}, extra: {sorted(seen - new_drawings)})")
    stats = slides["cost"]["stats"]
    report(f"cost stat 1 {stats[0]['value']!r} vs total_new {j['total_new']!r}", stats[0]["value"] == j["total_new"])
    report(f"cost stat 1 label has build_new {j['build_new']!r} and buy_new {j['buy_new']!r}", j["build_new"] in stats[0]["label"] and j["buy_new"] in stats[0]["label"])
    report(f"cost stat 2 {stats[1]['value']!r} vs total_all {j['total_all']!r}", stats[1]["value"] == j["total_all"])
    report(f"cost title {slides['cost']['title']!r} has total_new", j["total_new"] in slides["cost"]["title"])

    # buy_list: every costs.py buy item once, matched by name; cost range and source agree
    table = slides["buy_list"]["table"]
    ci = table["header"].index(COST_HEADER)
    fi = table["header"].index("From")
    by_name = {it["name"]: it for it in j["buy"]}
    seen = set()
    for row in table["rows"]:
        it = by_name.get(row[0])
        if it is None:
            report(f"buy_list {row[0]!r}", False, " (no costs.py buy item of this name)")
            continue
        seen.add(it["name"])
        want = usd_range(it["low"], it["high"])
        report(f"buy_list {row[0]!r}: deck {row[ci]!r}, from {row[fi]!r} vs costs.py {want!r}, {it['from_project'] or 'new'!r}", row[ci] == want and row[fi] == (it["from_project"] or "new"))
    report("buy_list covers every costs.py buy item", seen == set(by_name), f" (missing: {sorted(set(by_name) - seen)})")

    # every dollar range anywhere in either deck's text or notes (except the suppliers' price notes) is a costs.py figure
    known = {usd_range(it["low"], it["high"]) for it in items} | {j[k] for k in ("build_new", "buy_new", "total_new", "build_all", "buy_all", "total_all")}
    sys.path.insert(0, str(COSTS_PY.parent))
    import costs  # only for the job-shop hourly rate, which the JSON does not carry
    known.add(usd_range(*costs.JOB_SHOP_RATE_USD_PER_HOUR))
    for content, _ in DECKS:
        for s in json.load(open(content))["slides"]:
            if s["id"] == "suppliers":
                continue  # unit prices quoted from suppliers' pages, not costs.py figures
            for t in list(strings(s)) + [s["notes"]]:
                for r in RANGE.findall(t):
                    report(f"{Path(content).name} slide {s['id']}: dollar range {r!r}", r in known, "" if r in known else " (not a costs.py figure)")
    print(f"cost check: problems: {bad}")
    return bad


if __name__ == "__main__":
    total = sum(check(content_json, pptx) for content_json, pptx in DECKS)
    print(f"total problems: {total}")
    print("cost consistency (build deck vs docs/procedures/build/costs.py --json):")
    cost_bad = check_costs(DECKS[0][0])
    sys.exit(1 if total or cost_bad else 0)
