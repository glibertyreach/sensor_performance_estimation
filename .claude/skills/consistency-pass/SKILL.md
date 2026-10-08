---
name: consistency-pass
description: Find where derived documents (decks, slide content files, appendices, READMEs, captions, speaker notes, summaries) have drifted from the specification or procedure they restate. Use it whenever a user asks to check documents against each other, says two slides or sections "seem contradictory", asks whether a deck or summary still matches its source, wants a consistency or cross-reference review before a release, or has just rebuilt a document set and wants to know what disagrees with what. Run it after any edit to a governing document whose summaries were not rebuilt from it, even if the user only asks "does everything still agree?". Reports only; never edits.
---

# Consistency pass

`/consistency-pass <governing document> <derived document> [<derived document> ...]`

A document set describes one thing at different lengths: a full specification or
procedure, and shorter things made from it, such as decks, the content files that
decks are built from, appendices, READMEs, figure captions, speaker notes. Every time
the long document changes, or a summary is written from memory of it, a derived
statement can drift: an option is added that the specification excludes, a condition
is dropped, a number changes, a check is assigned to the wrong party. Each statement
looks fine on its own, so skimming never finds it. This pass finds it by comparing
every derived statement against the statement that governs it.

The mistake this catches, in its general form: a derived statement is attached to a
governing statement and does not agree with it. The specification governs; a derived
document can explain, shorten or add rationale, but it cannot change what is
specified. Fixing the asymmetry of authority once, before reading, is what makes the
pass work; without it a reviewer sees two plausible sentences and lets both stand.

## Procedure

1. **Identify the governing document.** It is the one the user names first, or the
   full specification or procedure when the user names only a set. If two documents
   both claim authority over the same item (a specification and a drawing, say), ask
   which governs rather than guessing, since every finding depends on it.

2. **Get the text of every document.** Run `scripts/extract_text.py` on each file;
   it writes plain text, into a temporary folder it prints (or `--out DIR`), never next
   to the inputs, for Markdown, JSON content files (every string, with its key
   path), Word files and PowerPoint files (slide text and speaker notes, slide by
   slide). Read the extracted text in full. Derived documents hide restatements in
   places a reader skips: speaker notes, figure captions, table footers, the "notes"
   fields of a content file, help text quoted in an appendix. All of it is in scope.

3. **Extract the governing statements.** From the governing document, list every
   requirement, limit, allowed set of options, number with its unit, named party
   responsible for an action, sequence of steps, and anything marked provisional
   (placeholder, to be confirmed, rough figure). Give each an identifier (G1, G2, ...)
   and keep the quoted text. Options lists need care: "A or B" is a closed set of
   two; "for example A" is open. Record which.

4. **Attach every derived statement.** Walk each derived document and attach each
   statement to the governing statement it restates. Three outcomes:
   - attached and agreeing: nothing to report;
   - attached and conflicting: a finding (step 5);
   - not attachable: note it in a separate list. Derived documents legitimately say
     things the specification does not (a rationale, a tip, a supplier's address).
     That list is for the user to skim, not a list of defects; do not pad the
     findings with it.

5. **Decide what counts as a conflict.** Report a derived statement that:
   - adds an option, material, method or source the governing set excludes;
   - drops a condition the governing statement imposes (a tolerance, a time, a
     prerequisite, a "after painting and mounting");
   - changes a number, a unit or a count, including rounding that crosses a limit;
   - names a different party for an action (supplier versus in-house, technician
     versus engineer);
   - reorders steps that the governing document orders, or merges steps it keeps
     apart;
   - states as settled what the governing text marks as provisional, or the reverse;
   - keeps a value the governing document has since changed (the commonest case
     after an edit).
   Treat a list of options, candidates or suppliers as a set of claims that each
   entry meets the requirement. An entry that cannot meet it is a conflict, not a
   convenience, even when the entry is sensible on its own. `references/conflict_kinds.md`
   gives one worked example of each kind.

6. **Where the governing document is silent, say so.** If a derived document states
   something the governing one neither requires nor excludes, that is a gap in the
   governing document, not a conflict; list it as a gap with a proposed sentence for
   the governing document, and do not pick a side.

7. **Report, do not edit.** The user decides what to change and where. Fixing one
   side silently would hide the decision; many conflicts are resolved by changing the
   governing document, not the derived one.

## Report

Use this structure:

```
# Consistency pass: <governing document> against <n> derived documents

## Conflicts
| # | Governing statement (id, quoted, location) | Conflicting statement (quoted, location) | Kind | Proposed fix |

## Gaps in the governing document
| # | Derived statement (quoted, location) | What the governing document should say |

## Not attachable (for a skim, not defects)
- <location>: <statement>

## Coverage
<which documents were read, which parts of them (slides, notes, captions), and
anything that could not be extracted>
```

Quote statements exactly, with a location precise enough to find in one step: a
section number, a slide number with "notes" or "table" or "caption", a JSON key
path, a line number for Markdown. Order conflicts by consequence: a wrong limit or
an excluded option before a wording difference. One row per conflict; a statement
that conflicts in two ways gets two rows. Keep proposed fixes to one sentence that
says which document changes.

## Scope and judgment

- Numbers that differ by presentation only ("0.05 mm" and "50 micrometers", "about
  25 GB" and "24 GB at 10 GB per 1,000 frames") are not conflicts; say nothing.
- Shortening is not dropping: "flat to 0.05 mm" on a slide whose notes carry "after
  painting and mounting" agrees with the specification. Read the notes before
  reporting a dropped condition.
- A derived document that paraphrases a step in different words agrees if a
  technician following either would do the same thing.
- When the set is large, work governing statement by governing statement rather than
  document by document, so that each requirement's mentions line up in one place.
- If a derived document is generated from the governing one by a script (a filled
  template, a cost table from a cost script), say so in Coverage and still check it:
  generation prevents drift only for the fields it generates.
