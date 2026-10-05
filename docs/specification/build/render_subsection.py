#!/usr/bin/env python3
# Copied from the document-review skill so the review tooling is self-contained; keep in step with the skill.
"""
render_subsection.py — render ONE subsection of a .docx as a one-page-style PDF,
with any embedded figures placed under their captions.

Usage:
    python render_subsection.py DOC.docx "7.2.2" out.pdf \
        [--header "Left text | Right text"] [--page "12"]

The heading is matched by prefix on the first token (e.g. "7.2.2", "1.1", "3").
Extraction runs from that heading until the next heading of the same or higher
level. Figures are read straight out of the .docx (word/media), so this works on
any Word document — no build pipeline required.

Requires: python-docx, weasyprint  (pip install python-docx weasyprint)
"""
import sys, re, os, base64, argparse
from docx import Document
from docx.oxml.ns import qn
from docx.text.paragraph import Paragraph
from docx.table import Table
from weasyprint import HTML

ap = argparse.ArgumentParser()
ap.add_argument("docx"); ap.add_argument("heading"); ap.add_argument("out")
ap.add_argument("--header", default="Document review")
ap.add_argument("--page", default="")
A = ap.parse_args()


# ---------------- LaTeX (.tex) path ----------------
if A.docx.lower().endswith(".tex"):
    import subprocess, tempfile, shutil
    txt = open(A.docx, encoding="utf-8", errors="ignore").read()
    m = re.search(r"\\begin\{document\}", txt)
    preamble = txt[:m.start()] if m else "\\documentclass{article}\n"
    body = txt[m.end():] if m else txt
    body = re.split(r"\\end\{document\}", body)[0]
    LEV = {"section":1,"subsection":2,"subsubsection":3,"paragraph":4}
    heads = [(mm.start(), LEV[mm.group(1)], mm.group(2))
             for mm in re.finditer(r"\\(section|subsection|subsubsection|paragraph)\*?\{([^}]*)\}", body)]
    tgt = None
    for i,(pos,lv,title) in enumerate(heads):
        if A.heading.lower() in title.lower():
            tgt = (i,pos,lv); break
    if tgt is None:
        sys.exit("Section title not found in .tex: " + A.heading)
    i,pos,lv = tgt
    end = len(body)
    for pos2,lv2,_ in heads[i+1:]:
        if lv2 <= lv: end = pos2; break
    frag = body[pos:end]
    texdir = os.path.dirname(os.path.abspath(A.docx))
    doc = (preamble + "\\begin{document}\n"
           + "\\graphicspath{{" + texdir + "/}}\n"
           + "\\pagestyle{empty}\n" + frag + "\n\\end{document}\n")
    with tempfile.TemporaryDirectory() as tmp:
        tf = os.path.join(tmp, "sub.tex"); open(tf,"w",encoding="utf-8").write(doc)
        for _ in range(2):
            subprocess.run(["pdflatex","-interaction=nonstopmode","-halt-on-error","-output-directory",tmp,tf],
                           cwd=texdir, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        pdf = os.path.join(tmp,"sub.pdf")
        if not os.path.exists(pdf):
            sys.exit("pdflatex failed (check the document preamble compiles standalone)")
        shutil.copy(pdf, A.out); print("wrote", A.out, "(LaTeX)"); sys.exit(0)

d = Document(A.docx)
# rId -> data-URI for every image in the package
rels = d.part.rels
def rid_to_uri(rid):
    try:
        part = rels[rid].target_part
        b64 = base64.b64encode(part.blob).decode()
        ext = (part.content_type.split("/")[-1] or "png").replace("jpeg","jpeg")
        return f"data:image/{ext};base64,{b64}"
    except Exception:
        return None

def lvl(p):
    n = p.style.name if (p.style and p.style.name) else ""
    if n.startswith("Heading 1"): return 1
    if n.startswith("Heading 2"): return 2
    if n.startswith("Heading 3"): return 3
    return 0

EMU_PER_POINT = 12700
"""Word stores picture extents in English Metric Units; 12,700 EMU make one point."""
TEX_COMMAND_PATTERN = r"\\[A-Za-z]+"
"""A TeX command in a picture's alt text marks it as a typeset formula, not a figure."""

def para_images(p):
    """(data-URI, width in points or None, is_formula) for every picture in the paragraph.
    The width is the extent Word records for the picture, so the page shows it at the
    size the document gives it rather than at the bitmap's pixel size."""
    pictures = []
    for drawing in p._p.findall(".//" + qn("w:drawing")):
        blip = drawing.find(".//" + qn("a:blip"))
        if blip is None:
            continue
        rid = blip.get(qn("r:embed")) or blip.get(qn("r:link"))
        u = rid_to_uri(rid) if rid else None
        if not u:
            continue
        extent = drawing.find(".//" + qn("wp:extent"))
        width_pt = int(extent.get("cx")) / EMU_PER_POINT if extent is not None and extent.get("cx") else None
        doc_pr = drawing.find(".//" + qn("wp:docPr"))
        descr = (doc_pr.get("descr") or "") if doc_pr is not None else ""
        pictures.append((u, width_pt, bool(re.search(TEX_COMMAND_PATTERN, descr))))
    return pictures

esc = lambda s: s.replace("&","&amp;").replace("<","&lt;").replace(">","&gt;")

ORDERED_FORMATS = {"decimal", "lowerLetter", "upperLetter", "lowerRoman", "upperRoman"}
"""Word numbering formats rendered as an ordered list; everything else is a bullet list."""

def list_numbering(p):
    """(kind, level, numId) of a list paragraph: kind is 'ol' or 'ul' from the numbering
    definition's format at the paragraph's level, level is the indent level (0 = outermost)."""
    num_pr = p._p.find(".//" + qn("w:numPr"))
    if num_pr is None:
        return "ul", 0, ""
    ilvl = num_pr.find(qn("w:ilvl")); num_id = num_pr.find(qn("w:numId"))
    level = int(ilvl.get(qn("w:val"))) if ilvl is not None else 0
    num_val = num_id.get(qn("w:val")) if num_id is not None else ""
    kind = "ul"
    try:
        numbering = d.part.numbering_part.element
        num = numbering.find(f".//{qn('w:num')}[@{qn('w:numId')}='{num_val}']")
        abstract_id = num.find(qn("w:abstractNumId")).get(qn("w:val"))
        abstract = numbering.find(f".//{qn('w:abstractNum')}[@{qn('w:abstractNumId')}='{abstract_id}']")
        lvl = abstract.find(f"{qn('w:lvl')}[@{qn('w:ilvl')}='{level}']")
        fmt = lvl.find(qn("w:numFmt")).get(qn("w:val"))
        kind = "ol" if fmt in ORDERED_FORMATS else "ul"
        # A list that continues an earlier one (after a formula or figure) carries its first
        # number as the level's start value; seed the running count from it once.
        start = lvl.find(qn("w:start"))
        override = num.find(f"{qn('w:lvlOverride')}[@{qn('w:ilvl')}='{level}']")
        if override is not None and override.find(qn("w:startOverride")) is not None:
            start = override.find(qn("w:startOverride"))
        if start is not None and (num_val, level) not in LIST_COUNTS:
            LIST_COUNTS[(num_val, level)] = int(start.get(qn("w:val"))) - 1
    except Exception:
        pass
    return kind, level, num_val

def group_list_items(items):
    """Nest consecutive <li> markers into <ol>/<ul> by level. A deeper list opens inside the
    open item (valid HTML, so the outer numbering is not disturbed); every ordered item carries
    its running number as a value attribute, so a Word list that resumes after an interruption
    keeps counting."""
    html = ""; stack = []  # each entry: [kind, level, item_open]
    for li in items:
        m = re.match(r'<li data-kind="(\w+)" data-level="(\d+)" data-num="([^"]*)">', li)
        kind, level, num_id = m.group(1), int(m.group(2)), m.group(3)
        text = li[m.end():-len("</li>")]
        while stack and stack[-1][1] > level:
            top = stack.pop()
            html += ("</li>" if top[2] else "") + f"</{top[0]}>"
        if not stack or stack[-1][1] < level:
            # An ordered list that resumes after a formula or figure in the same section continues
            # the numbering of the list that ended there, as the source document does.
            # (list_numbering may already have seeded this list's count from a start value of 1,
            # which carries no information, so a count of zero is treated as unseeded.)
            if kind == "ol" and LIST_COUNTS.get((num_id, level), 0) == 0 and level in CONTINUED_COUNTS:
                LIST_COUNTS[(num_id, level)] = CONTINUED_COUNTS[level]
            # The PDF engine (WeasyPrint) ignores both the <ol start> attribute and a per-item
            # value attribute, but honors a CSS counter reset, so the resumed count is carried as
            # an inline counter-reset on the <ol> (the value attribute is kept for readers of the
            # intermediate HTML).
            resumed = LIST_COUNTS.get((num_id, level), 0)
            start = f' style="counter-reset: list-item {resumed}"' if kind == "ol" and resumed else ""
            html += f"<{kind}{start}>"; stack.append([kind, level, False])
        elif stack[-1][2]:
            html += "</li>"; stack[-1][2] = False
        LIST_COUNTS[(num_id, level)] = LIST_COUNTS.get((num_id, level), 0) + 1
        if kind == "ol":
            CONTINUED_COUNTS[level] = LIST_COUNTS[(num_id, level)]
        value = f' value="{LIST_COUNTS[(num_id, level)]}"' if kind == "ol" else ""
        html += f"<li{value}>{text}"; stack[-1][2] = True
    while stack:
        top = stack.pop()
        html += ("</li>" if top[2] else "") + f"</{top[0]}>"
    return html

LIST_COUNTS = {}
"""Items emitted so far per (Word list id, level), so a resumed ordered list keeps counting."""
CONTINUED_COUNTS = {}
"""Last ordered-item number emitted per level since the last heading; a new ordered list in the
same section continues from it (a formula or figure between two parts of one list)."""

def para_html(p):
    imgs = para_images(p)
    t = p.text
    L = lvl(p)
    st = p.style.name if (p.style and p.style.name) else ""
    cap = re.match(r'(Figure|Fig\.|Table)\s', t.strip())
    if imgs:
        if all(is_formula for _, _, is_formula in imgs):
            tag = "".join(f'<img src="{u}"' + (f' style="width:{w:.1f}pt"' if w else "") + "/>" for u, w, _ in imgs)
            return f'<p class="formula">{tag}</p>'
        tag = "".join(f'<img src="{u}"' + (f' style="width:{w:.1f}pt"' if w else "") + "/>" for u, w, _ in imgs)
        capt = f'<figcaption>{esc(t)}</figcaption>' if t.strip() else ""
        return f'<figure>{tag}{capt}</figure>'
    if not t.strip(): return ""
    if L == 1: return f'<h1>{esc(t)}</h1>'
    if L == 2: return f'<h2>{esc(t)}</h2>'
    if L == 3: return f'<h3>{esc(t)}</h3>'
    if cap:    return f'<figcaption class="orphan">{esc(t)}</figcaption>'
    if "List" in st:
        kind, level, num_id = list_numbering(p)
        return f'<li data-kind="{kind}" data-level="{level}" data-num="{num_id}">{esc(t)}</li>'
    return f'<p>{esc(t)}</p>'

def table_html(tbl):
    rows = ""
    for ri, r in enumerate(tbl.rows):
        tag = "th" if ri == 0 else "td"
        rows += "<tr>" + "".join(f'<{tag}>{esc(c.text)}</{tag}>' for c in r.cells) + "</tr>"
    return f'<table class="tbl">{rows}</table>'

# locate start heading
paras = d.paragraphs
start_p = None; startlvl = None
for p in paras:
    if lvl(p) and (p.text.strip().split()[:1] or [""])[0].rstrip(".") == A.heading:
        start_p = p._p; startlvl = lvl(p); break
if start_p is None:
    sys.exit(f"Heading not found: {A.heading}")

parts = []; inside = False
for child in d.element.body.iterchildren():
    if child.tag == qn("w:p"):
        p = Paragraph(child, d)
        if p._p is start_p: inside = True
        elif inside and lvl(p) and lvl(p) <= startlvl: break
        if inside: parts.append(para_html(p))
    elif child.tag == qn("w:tbl") and inside:
        parts.append(table_html(Table(child, d)))

# group consecutive <li> markers into nested <ol>/<ul>
body = ""; buf = []
for h in parts:
    if h.startswith("<li "): buf.append(h)
    else:
        if buf: body += group_list_items(buf); buf = []
        if h.startswith("<h"): CONTINUED_COUNTS.clear()
        body += h
if buf: body += group_list_items(buf)

L, R = (A.header.split("|") + [""])[:2] if "|" in A.header else (A.header, "")
foot = f'@bottom-center {{ content: "{A.page}"; font-family: Georgia, serif; font-size: 9pt; color:#999; }}' if A.page else ""
CSS = f'''
@page {{ size: Letter; margin: 1in 1in; {foot} }}
body {{ font-family: Georgia,'Times New Roman',serif; font-size:11pt; line-height:1.55; color:#1a1a1a; }}
.hdr {{ display:flex; justify-content:space-between; font-family:Arial,sans-serif; font-size:8pt; letter-spacing:.06em; color:#8a8a8a; text-transform:uppercase; border-bottom:.5pt solid #ddd; padding-bottom:5pt; margin-bottom:22pt; }}
h1 {{ font-size:15pt; margin:0 0 12pt; }} h2 {{ font-size:12.5pt; margin:14pt 0 9pt; }} h3 {{ font-size:11.5pt; margin:12pt 0 7pt; }}
p {{ text-align:justify; margin:0 0 11pt; }}
ul, ol {{ margin:0 0 11pt 0; padding-left:20pt; }} li > ol, li > ul {{ margin:6pt 0 0 0; }} li {{ margin:0 0 6pt; text-align:justify; }}
figure {{ margin:14pt 0; text-align:center; page-break-inside:avoid; }}
figure img {{ max-width:100%; max-height:340pt; border:.5pt solid #ccc; }}
p.formula {{ text-align:center; margin:4pt 0 11pt; }} p.formula img {{ max-width:100%; }}
figcaption {{ font-size:9pt; color:#555; font-style:italic; text-align:justify; margin-top:5pt; }}
.tbl {{ border-collapse:collapse; width:100%; font-family:Arial,sans-serif; font-size:8.5pt; margin:8pt 0 12pt; }}
.tbl th,.tbl td {{ border:.5pt solid #bbb; padding:3pt 5pt; text-align:left; vertical-align:top; }}
.tbl th {{ background:#f0f0f0; font-weight:bold; }}
'''
doc = f'''<!doctype html><html><head><meta charset="utf-8"><style>{CSS}</style></head><body>
<div class="hdr"><span>{esc(L.strip())}</span><span>{esc(R.strip())}</span></div>
{body}
</body></html>'''
import os as _os
if _os.environ.get("RENDER_DEBUG_HTML"): open(_os.environ["RENDER_DEBUG_HTML"], "w").write(doc)
HTML(string=doc).write_pdf(A.out)
print("wrote", A.out, "| figures:", sum(1 for h in parts if h.startswith("<figure")))
