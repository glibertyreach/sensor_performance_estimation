# Building the VSX3000 performance-testing decks

`make_perf_deck.js` builds two decks from two content files (all slide text and
speaker notes) and the figures and drawings in `../assets/`:

| Content file | Deck |
|---|---|
| `perf_build_deck_content.json` | `../vsx3000_procurement_build.pptx` (17 slides) |
| `perf_procedure_deck_content.json` | `../vsx3000_test_procedure.pptx` (20 slides) |

Each slide of a content file has an "id", a "builder" (the layout function in the
script that draws it; the script falls back to the id when there is no "builder"),
a "layout" (`title_dark` or `title_only`), a "section", a "title", "notes" and the
fields its builder reads. Several slides share a builder: `fixtures` and `runout`
(figure card with icon cards), `board` (figure beside check rows), `approach`
(figure beside numbered steps), `plate_spec` (table; the plate drawing appears only
beside a two-column table), `checks`, `loop`, `robot_program`, `purpose`/`prep`, and
`pair` (two drawings side by side at one height, the caption under them). Edit a JSON
to change wording; edit the constants at the top of the script to change layout,
sizes or colors. Figure pixel sizes are read when the script runs, so a re-cropped
figure needs no code change. The generator was adapted from the registration
generator of `plane_plane_registration_ro`, itself adapted from the stage-1
generator of `depth_calibration_from_spherical_target`; every constant or builder
that differs from the registration one carries a comment saying why.

A drawing file that does not exist yet is not an error: the build prints a NOTE and
draws a labeled placeholder box ("PT-07: drawing pending") in its slot. Make the
drawing, run `make_assets.py` and build again.

1. Install the Node dependencies once, in this folder: `npm install`
2. From the repository root, build both decks:

   ```
   NODE_PATH=docs/presentations/build/node_modules \
   PPTX_SKILL_SCRIPTS=<folder holding apply_theme.js> \
       node docs/presentations/build/make_perf_deck.js
   ```

   or one deck, giving the content file and the output file:

   ```
   NODE_PATH=docs/presentations/build/node_modules \
   PPTX_SKILL_SCRIPTS=<folder holding apply_theme.js> \
       node docs/presentations/build/make_perf_deck.js \
           docs/presentations/build/perf_procedure_deck_content.json \
           docs/presentations/vsx3000_test_procedure.pptx
   ```

   The script prints a WARNING when a table, a column of cards or a list needs more
   room than its slide gives it. It must print none.

3. Check the text: `python3 docs/presentations/archive/check_perf_content.py`
   (needs `markitdown[pptx]` and `python-pptx`) compares both decks with their
   JSON files (every string on its slide, notes equal to the JSON notes) and then
   checks the build deck's cost figures (the cost slide's table, stats and title,
   the build_list and buy_list rows, and every dollar range in either deck) against
   `python3 docs/procedures/build/costs.py --json`, their single source. It must
   report 0 problems and a passing cost check. After a change to `costs.py` or to
   a content file, rebuild first.

4. Check the layout without rendering:
   `python3 docs/presentations/archive/check_perf_layout.py` warns about
   overlapping text boxes and text that the generator's own text-width model says
   will not fit its box. It estimates; a render is the final judge. Render checks
   (PDF, images) go in `../archive/qa_renders/<deck>/`:

   ```
   soffice --headless --convert-to pdf --outdir docs/presentations/archive/qa_renders/<deck> docs/presentations/<deck>.pptx
   pdftoppm -jpeg -r 60 docs/presentations/archive/qa_renders/<deck>/<deck>.pdf docs/presentations/archive/qa_renders/<deck>/s
   ```

   (LibreOffice needs the Impress component and metric-compatible fonts for
   Calibri and Cambria, for example Carlito and Caladea, or the render is wrong.)

5. Validate the package structure of each finished deck:
   `python3 <skill folder>/scripts/office/validate.py docs/presentations/<deck>.pptx`

`apply_theme.js` comes from the pptx skill used to write this generator; it writes
the theme's colors and fonts into the finished file. The script needs it to run:
point `PPTX_SKILL_SCRIPTS` at the folder holding a copy, or keep the script's
default path. The same skill's `scripts/office/validate.py` checks a finished deck's
package structure.

## Regenerating the figures and drawings

The files in `../assets/` are copies of the procedure's figures in
`docs/procedures/figures/` and drawings in `docs/procedures/drawings/`. Regenerate
them, from the repository root, with

```
python3 docs/presentations/build/make_assets.py
```

(needs `Pillow` and `numpy`). Regenerate the figures and drawings first if one
changes. Whole figures (`targets`, `chamfer`, `mounting`, `setup`, `plan_example`,
`zstep`) and the seven drawings (`PT-01` to `PT-07`, 3300 x 2100 px) are copied as
they are. The one cropped file, `stations_left.png`, is the left panel of
`fig_stations.png`: the script erases the panel title, crops the panel to its content
and restores a 20 px margin on every side; the split column and the title box are
named constants at the top of the script, in pixels of the source figure. If that
figure's layout changes, view the output and adjust the constants. The script also
removes any file in `../assets/` that neither content file references, and skips a
drawing that does not exist yet with a message.
