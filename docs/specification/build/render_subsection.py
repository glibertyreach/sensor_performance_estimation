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

def para_images(p):
    uris = []
    for blip in p._p.findall(".//" + qn("a:blip")):
        rid = blip.get(qn("r:embed")) or blip.get(qn("r:link"))
        if rid:
            u = rid_to_uri(rid)
            if u: uris.append(u)
    return uris

esc = lambda s: s.replace("&","&amp;").replace("<","&lt;").replace(">","&gt;")

def para_html(p):
    imgs = para_images(p)
    t = p.text
    L = lvl(p)
    st = p.style.name if (p.style and p.style.name) else ""
    cap = re.match(r'(Figure|Fig\.|Table)\s', t.strip())
    if imgs:
        tag = "".join(f'<img src="{u}"/>' for u in imgs)
        capt = f'<figcaption>{esc(t)}</figcaption>' if t.strip() else ""
        return f'<figure>{tag}{capt}</figure>'
    if not t.strip(): return ""
    if L == 1: return f'<h1>{esc(t)}</h1>'
    if L == 2: return f'<h2>{esc(t)}</h2>'
    if L == 3: return f'<h3>{esc(t)}</h3>'
    if cap:    return f'<figcaption class="orphan">{esc(t)}</figcaption>'
    if "List" in st: return f'<li>{esc(t)}</li>'
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

# group <li> into <ul>
body = ""; buf = []
for h in parts:
    if h.startswith("<li>"): buf.append(h)
    else:
        if buf: body += "<ul>" + "".join(buf) + "</ul>"; buf = []
        body += h
if buf: body += "<ul>" + "".join(buf) + "</ul>"

L, R = (A.header.split("|") + [""])[:2] if "|" in A.header else (A.header, "")
foot = f'@bottom-center {{ content: "{A.page}"; font-family: Georgia, serif; font-size: 9pt; color:#999; }}' if A.page else ""
CSS = f'''
@page {{ size: Letter; margin: 1in 1in; {foot} }}
body {{ font-family: Georgia,'Times New Roman',serif; font-size:11pt; line-height:1.55; color:#1a1a1a; }}
.hdr {{ display:flex; justify-content:space-between; font-family:Arial,sans-serif; font-size:8pt; letter-spacing:.06em; color:#8a8a8a; text-transform:uppercase; border-bottom:.5pt solid #ddd; padding-bottom:5pt; margin-bottom:22pt; }}
h1 {{ font-size:15pt; margin:0 0 12pt; }} h2 {{ font-size:12.5pt; margin:14pt 0 9pt; }} h3 {{ font-size:11.5pt; margin:12pt 0 7pt; }}
p {{ text-align:justify; margin:0 0 11pt; }}
ul {{ margin:0 0 11pt 0; padding-left:20pt; }} li {{ margin:0 0 6pt; text-align:justify; }}
figure {{ margin:14pt 0; text-align:center; page-break-inside:avoid; }}
figure img {{ max-width:100%; max-height:340pt; border:.5pt solid #ccc; }}
figcaption {{ font-size:9pt; color:#555; font-style:italic; text-align:justify; margin-top:5pt; }}
.tbl {{ border-collapse:collapse; width:100%; font-family:Arial,sans-serif; font-size:8.5pt; margin:8pt 0 12pt; }}
.tbl th,.tbl td {{ border:.5pt solid #bbb; padding:3pt 5pt; text-align:left; vertical-align:top; }}
.tbl th {{ background:#f0f0f0; font-weight:bold; }}
'''
doc = f'''<!doctype html><html><head><meta charset="utf-8"><style>{CSS}</style></head><body>
<div class="hdr"><span>{esc(L.strip())}</span><span>{esc(R.strip())}</span></div>
{body}
</body></html>'''
HTML(string=doc).write_pdf(A.out)
print("wrote", A.out, "| figures:", sum(1 for h in parts if h.startswith("<figure")))
