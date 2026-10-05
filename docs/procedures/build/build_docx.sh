#!/usr/bin/env bash
# Regenerate a Word version of a Markdown document of this repository.
#   build_docx.sh <markdown file> "<title>"
# With no arguments, builds the stage-1 procedure and the analysis document.
# Requires pandoc (the pip package pypandoc_binary bundles one). Figures get
# the text width through build/image_width.lua.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
build() {
  local source="$1" title="$2"
  local dir; dir="$(cd "$(dirname "$source")" && pwd)"
  local base; base="$(basename "$source" .md)"
  ( cd "$dir" && python3 - "$base" "$title" "$HERE/image_width.lua" <<'PY'
import sys, pathlib, pypandoc
base, title, lua = sys.argv[1:4]
pypandoc.convert_file(f"{base}.md", "docx", outputfile=f"{base}.docx",
                      extra_args=["--resource-path=.", f"--lua-filter={lua}", "--metadata", f"title={title}",
                                  "--toc", "--toc-depth=1"])
# pandoc omits the PNG default content type that strict validators expect;
# declare it so the file passes validation without a repair step.
import zipfile, shutil
path = f"{base}.docx"; tmp = path + ".tmp"
with zipfile.ZipFile(path) as zin:
    ct = zin.read("[Content_Types].xml").decode()
    if 'Extension="png"' not in ct:
        ct = ct.replace("<Default ", '<Default Extension="png" ContentType="image/png"/><Default ', 1)
        with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as zout:
            for item in zin.infolist():
                zout.writestr(item, ct.encode() if item.filename == "[Content_Types].xml" else zin.read(item.filename))
if pathlib.Path(tmp).exists():
    shutil.move(tmp, path)
print(f"wrote {base}.docx")
PY
  )
}
if [ "$#" -eq 2 ]; then
  build "$1" "$2"
else
  ROOT="$(cd "$HERE/../../.." && pwd)"
  build "$ROOT/docs/procedures/stage1_capture_procedure.md" "Stage-1 Calibration Captures: Step-by-Step Procedure"
  build "$ROOT/docs/analysis/calibration_from_spherical_target_analysis.md" "Depth and Normal Correction from Spherical and Planar Targets: Problem Analysis"
fi
