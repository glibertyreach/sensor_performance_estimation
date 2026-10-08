"""Single source of the cost figures quoted in the performance-testing procedure and its decks.

Every estimate is a range in US dollars, as of ESTIMATE_DATE. Where an item comes from the stage-1
calibration project or the registration project it is marked so and costs nothing more when those
projects have been run. The machining figures assume typical United States job-shop rates
(JOB_SHOP_RATE_USD_PER_HOUR) on aluminum tooling plate and small turned steel parts; the finish
figures assume bead blasting by the same shop. Treat every figure as plus or minus
UNCERTAINTY_PERCENT and get quotes. The procedure builder (build.py) fills the markers
{{COST_TABLE_BUILD}}, {{COST_TABLE_BUY}} and {{COST_PARAGRAPH}} from this module, and the deck
content reads the same figures as JSON:

    python3 docs/procedures/build/costs.py            # Markdown tables and paragraph
    python3 docs/procedures/build/costs.py --json     # the same figures as JSON
"""
from __future__ import annotations

import json
import sys
from dataclasses import dataclass, asdict

ESTIMATE_DATE = "October 2026"
UNCERTAINTY_PERCENT = 50
"""Every figure is plus or minus this much."""
JOB_SHOP_RATE_USD_PER_HOUR = (80, 150)
"""Basis of the machining estimates."""
LEAST_CERTAIN_ITEM = "the four two-plane targets"
"""Named in the cost paragraph as the figure most likely to move at quotation."""


@dataclass(frozen=True)
class CostItem:
    name: str
    quantity: str
    low: int
    high: int
    purpose: str
    drawing: str = ""
    """Drawing number(s) for a built item; empty for a bought one."""
    from_project: str = ""
    """'stage 1' or 'registration' when the item already exists from that project; empty otherwise."""


# ---------------------------------------------------------------------------
# What must be built (section 1b). Drawing numbers are the PT-xx sheets of appendix F.
# ---------------------------------------------------------------------------
BUILD_ITEMS = (
    CostItem("Target adapter", "1", 300, 700,
             "Flange plate with the spigot bore and the cross-pin hole; ISO 9409-1-50-4-M6 interface", "PT-01"),
    CostItem("Target spigots", "4", 240, 600,
             "One per feature target: turned spigot with its orientation dowel and mounting flange", "PT-02"),
    CostItem("Standoff sets, 15 mm and 60 mm gaps", "3 sets of 8, plus 4 spares", 120, 360,
             "Set the gap G: between the front and back plates of T3b and T5, and hidden behind the raised square of T3a;"
             " T4's disks stand on their own 2 mm posts", "PT-03"),
    CostItem("T3a, raised square", "1", 350, 900,
             "Back plate, knife-edged 160 mm square on hidden standoffs, back-beveled; bead-blast finish", "PT-04"),
    CostItem("T3b, square window", "1", 350, 900,
             "Front plate with the countersunk 160 mm window, back plate; bead-blast finish", "PT-05"),
    CostItem("T4, disk plate", "1", 450, 1200,
             "Back plate, three back-beveled disks on 2 mm posts (one assembly per gap), one post-only site;"
             " bead-blast finish", "PT-06"),
    CostItem("T5, cutout plate", "1", 450, 1100,
             "Front plate with three countersunk holes, removable back plate, and the edge bracket that carries"
             " the spigot; bead-blast finish", "PT-07"),
    CostItem("Noise and registration board (T2) with its adapter", "1", 300, 1000,
             "200 x 150 mm board flat to 0.05 mm on the SC1-05 board adapter", "SC1-05", "stage 1"),
    CostItem("Run-out fixture", "1", 240, 570,
             "Dial indicator on a magnetic base on a bolted steel plate; the in-house flatness check", "none needed",
             "stage 1"),
    CostItem("Rigid sensor mount", "1", 0, 0,
             "Stiff bracket from the existing drawing of previous work; not estimated here", "existing drawing",
             "stage 1"),
)

# ---------------------------------------------------------------------------
# What must be bought (section 1c).
# ---------------------------------------------------------------------------
BUY_ITEMS = (
    CostItem("Ball-lock pins, 8 mm, 2 off", "2", 40, 120,
             "Hold a target's spigot in the adapter; one spare"),
    CostItem("Calipers, 600 mm, or a 500 mm steel rule", "1", 40, 250,
             "Plate width and length, the in-house size check"),
    CostItem("Calipers with depth rod, 150 mm", "1", 30, 150,
             "Gap G and standoff lengths", "", "stage 1"),
    CostItem("Torque wrench", "1", 80, 250,
             "Recorded adapter and spigot screw torques", "", "stage 1"),
    CostItem("Temperature loggers, 2", "2", 60, 200,
             "Sensor housing and air, one sample per minute, for drift attribution"),
    CostItem("Consumables", "", 40, 120,
             "Medium-strength thread locker, low-strength removable retaining compound for the disk posts, isopropyl alcohol and wipes, padded cases for the targets"),
)
IN_HAND_ITEMS = (
    ("Phone camera", "Setup photos for the deliverables (section 13)"),
)


def total(items) -> tuple[int, int]:
    return sum(i.low for i in items), sum(i.high for i in items)


def total_new(items) -> tuple[int, int]:
    """Totals for the items that do not already exist from a related project."""
    return total([i for i in items if not i.from_project])


def usd(value: int) -> str:
    return f"${value:,}"


def usd_range(low: int, high: int) -> str:
    return "not estimated" if (low, high) == (0, 0) else f"{usd(low)} to {usd(high)}"


def build_table() -> str:
    rows = ["| Item | Quantity | Description | Drawing | Estimated cost (USD) | From |",
            "|---|---|---|---|---|---|"]
    for i in BUILD_ITEMS:
        rows.append(f"| {i.name} | {i.quantity} | {i.purpose} | {i.drawing} | {usd_range(i.low, i.high)} | "
                    f"{i.from_project or 'new'} |")
    return "\n".join(rows)


def buy_table() -> str:
    rows = ["| Item | Purpose | Estimated cost (USD) | From |", "|---|---|---|---|"]
    for i in BUY_ITEMS:
        rows.append(f"| {i.name} | {i.purpose} | {usd_range(i.low, i.high)} | {i.from_project or 'new'} |")
    for name, purpose in IN_HAND_ITEMS:
        rows.append(f"| {name} | {purpose} | in hand | in hand |")
    return "\n".join(rows)


def cost_paragraph() -> str:
    b_new = total_new(BUILD_ITEMS); y_new = total_new(BUY_ITEMS)
    b_all = total(BUILD_ITEMS); y_all = total(BUY_ITEMS)
    return (f"Cost estimates ({ESTIMATE_DATE}): the new items come to {usd_range(*b_new)} to build and "
            f"{usd_range(*y_new)} to buy, {usd_range(b_new[0] + y_new[0], b_new[1] + y_new[1])} in all. If the "
            f"stage-1 session has not been run, add its board, board adapter and run-out fixture and its calipers "
            f"and torque wrench: {usd_range(*b_all)} to build and {usd_range(*y_all)} to buy, "
            f"{usd_range(b_all[0] + y_all[0], b_all[1] + y_all[1])} in all. The figures come from suppliers' list "
            f"prices where these exist (appendix E) and otherwise from typical United States job-shop rates of about "
            f"{usd(JOB_SHOP_RATE_USD_PER_HOUR[0])} to {usd(JOB_SHOP_RATE_USD_PER_HOUR[1])} per hour. Treat them as "
            f"plus or minus {UNCERTAINTY_PERCENT} percent and get quotes; {LEAST_CERTAIN_ITEM} are the least certain "
            f"figures, because the knife edges, the back bevels and the bead-blast finish are each a shop's judgment.")


def as_json() -> dict:
    b_new = total_new(BUILD_ITEMS); y_new = total_new(BUY_ITEMS)
    b_all = total(BUILD_ITEMS); y_all = total(BUY_ITEMS)
    return {"estimate_date": ESTIMATE_DATE, "uncertainty_percent": UNCERTAINTY_PERCENT,
            "build": [asdict(i) for i in BUILD_ITEMS], "buy": [asdict(i) for i in BUY_ITEMS],
            "in_hand": [{"name": n, "purpose": p} for n, p in IN_HAND_ITEMS],
            "build_new": usd_range(*b_new), "buy_new": usd_range(*y_new),
            "total_new": usd_range(b_new[0] + y_new[0], b_new[1] + y_new[1]),
            "build_all": usd_range(*b_all), "buy_all": usd_range(*y_all),
            "total_all": usd_range(b_all[0] + y_all[0], b_all[1] + y_all[1])}


if __name__ == "__main__":
    if "--json" in sys.argv:
        print(json.dumps(as_json(), indent=1))
    else:
        print(build_table()); print(); print(buy_table()); print(); print(cost_paragraph())
