#!/usr/bin/env python3
"""
bridge_triage.py  --  Risk-based triage of Massachusetts bridges for underwater
(scour) inspection, for the Hackachusetts R6 "Safer inspection of critical
infrastructure" challenge.

Input : FHWA National Bridge Inventory 2022, Massachusetts (data/nbi/MA22.txt).
        Comma-delimited, single-quote quotechar, latin-1 encoding.

Output: a transparent 0-100 risk score per bridge (weighted sum of condition,
        scour, age, traffic exposure and underwater-inspection overdue-ness),
        plain-English reasons, a CLI with --top / --json / --summary, and a
        NBI -> SNBI field mapping helper (snbi_fields).

Dependencies: pandas, numpy only (plus the standard library).

The scoring is deliberately simple and auditable: every component is a 0..1
sub-score, multiplied by a weight from the WEIGHTS dict below, summed, and
scaled to 0..100. Nothing is hidden. Change the weights in one place.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from datetime import date

import numpy as np
import pandas as pd

# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #

# Path to the NBI file, relative to the repo root (this file lives in ai/).
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_NBI_PATH = os.path.join(_REPO_ROOT, "data", "nbi", "MA22.txt")

# "Today" for overdue calculations. The challenge framing is 2026-10.
TODAY = date(2026, 10, 1)   # NOTE: "overdue" = time since the last underwater date IN THE 2022 FILE; newer dives are not in this file

# Risk-score weights. They are summed, so the relative magnitude is what
# matters; the final score is rescaled to 0..100 by the sum of weights used.
# Each component below produces a 0..1 sub-score before weighting.
WEIGHTS = {
    "condition": 30.0,   # worst of deck/superstructure/substructure (58/59/60)
    "scour": 30.0,       # scour critical rating (item 113)
    "overdue": 20.0,     # underwater inspection overdue vs 92B interval + 93B
    "exposure": 12.0,    # traffic exposure, log(ADT) (item 29)
    "age": 8.0,          # age from YEAR_BUILT (item 27)
}

# NBI condition rating scale is 0 (failed) .. 9 (excellent). "N" = not
# applicable. We treat these NBI items as the drivers of the condition score.
_CONDITION_FIELDS = ["DECK_COND_058", "SUPERSTRUCTURE_COND_059", "SUBSTRUCTURE_COND_060"]

# SERVICE_UND_042B codes 5-9 mean the bridge carries something over water
# (waterway underneath) per the NBI coding guide.
_OVER_WATER_CODES = {"5", "6", "7", "8", "9"}


# --------------------------------------------------------------------------- #
# Parsing helpers
# --------------------------------------------------------------------------- #

def parse_coordinate(raw: str, is_longitude: bool) -> float | None:
    """
    Convert an NBI packed DDMMSSss coordinate to signed decimal degrees.

    NBI items 016 (lat) and 017 (long) are stored as digits DD MM SS ss where
    the last two digits are hundredths of a second. Latitude is 8 digits
    (DDMMSSss); longitude is 9 digits (DDDMMSSss) because degrees can reach
    three digits. Massachusetts longitudes are west, so the result is negated.

    Returns None when the value is missing or zero (NBI uses 0 for "unknown").
    """
    if raw is None:
        return None
    s = str(raw).strip()
    if not s or not s.isdigit():
        return None
    # All-zero means unknown in the NBI file.
    if int(s) == 0:
        return None

    # Longitude uses 3 degree-digits, latitude uses 2. Zero-pad defensively so
    # both parse the same way (seconds/minutes are always the last 6 digits).
    deg_digits = 3 if is_longitude else 2
    s = s.zfill(deg_digits + 6)

    degrees = int(s[:deg_digits])
    minutes = int(s[deg_digits:deg_digits + 2])
    seconds = int(s[deg_digits + 2:deg_digits + 4])
    hundredths = int(s[deg_digits + 4:deg_digits + 6])

    decimal = degrees + minutes / 60.0 + (seconds + hundredths / 100.0) / 3600.0
    if is_longitude:
        decimal = -decimal  # western hemisphere
    return round(decimal, 6)


def _to_float(raw, default=None):
    """Lenient float parse: returns default for blanks / non-numeric codes."""
    if raw is None:
        return default
    s = str(raw).strip()
    if s == "" or s.lower() == "nan":
        return default
    try:
        return float(s)
    except ValueError:
        return default


def _to_int(raw, default=None):
    f = _to_float(raw, None)
    return default if f is None else int(f)


def parse_underwater_interval_months(code_92b) -> int | None:
    """
    Parse NBI item 092B (UNDWATER_LOOK_SEE_092B).

    Format is a single Y/N flag optionally followed by the inspection interval
    in months, e.g. 'Y36', 'Y12', 'N  '. Returns the interval in months when
    an underwater inspection is required, else None.
    """
    if code_92b is None:
        return None
    s = str(code_92b).strip()
    if not s or s[0].upper() != "Y":
        return None
    digits = s[1:].strip()
    if digits.isdigit() and int(digits) > 0:
        return int(digits)
    # Required but no interval given: assume the common 36-month cycle.
    return 36


def parse_last_underwater_date(code_93b) -> date | None:
    """
    Parse NBI item 093B (UNDWATER_LAST_DATE_093B), stored as MMYY.

    Returns a date on the first of that month, or None if unparseable.
    Two-digit years are interpreted as 19xx/20xx with a sensible cutoff.
    """
    if code_93b is None:
        return None
    s = str(code_93b).strip()
    if s.lower() == "nan":
        return None
    # Normalise to 4 digits MMYY.
    s = s.split(".")[0]  # guard against '0819.0' style floats
    if not s.isdigit() or len(s) != 4:
        return None
    month = int(s[:2])
    yy = int(s[2:])
    if not (1 <= month <= 12):
        return None
    year = 2000 + yy if yy <= 70 else 1900 + yy
    return date(year, month, 1)


def months_between(d0: date, d1: date) -> int:
    """Whole months from d0 to d1 (d1 - d0), can be negative."""
    return (d1.year - d0.year) * 12 + (d1.month - d0.month)


# --------------------------------------------------------------------------- #
# Loading
# --------------------------------------------------------------------------- #

def load_nbi(path: str = DEFAULT_NBI_PATH) -> pd.DataFrame:
    """
    Load the NBI file as strings (every column read as text so we control the
    parsing of each field explicitly). Returns a pandas DataFrame.
    """
    df = pd.read_csv(
        path,
        sep=",",
        quotechar="'",
        encoding="latin-1",
        dtype=str,
        low_memory=False,
    )
    # Trim whitespace on the handful of fields we rely on (fixed-width padding).
    for col in df.columns:
        if df[col].dtype == object:
            df[col] = df[col].str.strip()
    return df


# --------------------------------------------------------------------------- #
# Sub-score functions (each returns a float in 0..1)
# --------------------------------------------------------------------------- #

def _condition_subscore(row) -> tuple[float, int | None]:
    """
    Worst (lowest) of deck/superstructure/substructure ratings -> 0..1 where
    poor condition is high risk. Rating 0 -> 1.0, rating 9 -> 0.0. 'N' ignored.
    Returns (subscore, lowest_rating_or_None).
    """
    ratings = []
    for f in _CONDITION_FIELDS:
        v = _to_int(row.get(f), None)
        if v is not None:
            ratings.append(v)
    if not ratings:
        return 0.0, None
    lowest = min(ratings)
    # Map 0..9 condition to 1..0 risk.
    sub = max(0.0, min(1.0, (9 - lowest) / 9.0))
    return sub, lowest


def _scour_subscore(row) -> tuple[float, str | None]:
    """
    Scour critical rating, NBI item 113. Codes 0-3 are scour critical (high
    risk), 4-9 are acceptable, and N/U/T are special (no numeric rating).

    Mapping (lower code = worse):
        0 -> 1.00 (failed / closed)      3 -> 0.70
        4 -> 0.45 (stable, countermeasures / action taken)
        5-9 -> taper down toward 0.0
        U (unknown) -> moderate 0.5 (deserves attention, we don't know)
        N (not over water) / T (tidal, low risk) -> 0.0
    Returns (subscore, raw_code).
    """
    raw = row.get("SCOUR_CRITICAL_113")
    code = None if raw is None else str(raw).strip()
    if not code:
        return 0.0, None
    if code.isdigit():
        n = int(code)
        if n <= 3:
            return {0: 1.0, 1: 0.9, 2: 0.8, 3: 0.7}[n], code
        # 4..9 acceptable, still a mild gradient (4 worse than 9).
        return max(0.0, (9 - n) / 10.0), code
    # Non-numeric special codes.
    if code.upper() == "U":   # unknown -> worth inspecting
        return 0.5, code
    # 'N' not applicable / not over water, 'T' tidal (assessed, low) -> 0.
    return 0.0, code


def _age_subscore(row) -> tuple[float, int | None]:
    """
    Age from YEAR_BUILT (item 27). Older = higher risk. We saturate at 100
    years: a bridge >=100 years old gets 1.0, a brand-new one 0.0.
    Returns (subscore, age_years_or_None).
    """
    yb = _to_int(row.get("YEAR_BUILT_027"), None)
    if yb is None or yb < 1700 or yb > TODAY.year:
        return 0.0, None
    age = TODAY.year - yb
    return max(0.0, min(1.0, age / 100.0)), age


def _exposure_subscore(row) -> tuple[float, int | None]:
    """
    Traffic exposure from ADT (item 29) on a log scale. 0 ADT -> 0.0, and we
    saturate near 100k vehicles/day (log10(1e5) = 5). Returns (subscore, adt).
    """
    adt = _to_int(row.get("ADT_029"), None)
    if adt is None or adt <= 0:
        return 0.0, adt
    sub = math.log10(adt) / 5.0  # log10(100000) == 5 -> 1.0
    return max(0.0, min(1.0, sub)), adt


def _overdue_subscore(row) -> tuple[float, dict]:
    """
    Underwater inspection overdue-ness from items 092B (interval) and 093B
    (last date) vs TODAY.

    due_date = last_date + interval_months. If overdue, score ramps from 0 at
    the due date to 1.0 one full interval past due. If no underwater inspection
    is required (92B not 'Y'), subscore is 0. If required but the last date is
    missing, treat as overdue (0.8) because we cannot confirm it was done.

    Returns (subscore, info) where info carries interval/last/due/overdue flags
    for the summary and reasons.
    """
    info = {
        "required": False,
        "interval_months": None,
        "last_date": None,
        "due_date": None,
        "months_overdue": None,
        "overdue": False,
    }
    interval = parse_underwater_interval_months(row.get("UNDWATER_LOOK_SEE_092B"))
    if interval is None:
        return 0.0, info  # no underwater inspection required

    info["required"] = True
    info["interval_months"] = interval

    last = parse_last_underwater_date(row.get("UNDWATER_LAST_DATE_093B"))
    if last is None:
        # Required but unknown last date -> cannot verify compliance.
        info["overdue"] = True
        return 0.8, info

    info["last_date"] = last.isoformat()
    # due_date = last + interval months.
    total = last.month - 1 + interval
    due = date(last.year + total // 12, total % 12 + 1, 1)
    info["due_date"] = due.isoformat()

    months_over = months_between(due, TODAY)
    info["months_overdue"] = months_over
    if months_over <= 0:
        return 0.0, info  # not yet due
    info["overdue"] = True
    # Ramp 0..1 across one interval past the due date.
    return max(0.0, min(1.0, months_over / float(interval))), info


# --------------------------------------------------------------------------- #
# Scoring
# --------------------------------------------------------------------------- #

def score_row(row) -> dict:
    """
    Compute the 0..100 risk score and top-3 reasons for one bridge row.

    Returns a dict with the score, the per-component contributions, the top
    reasons, decoded lat/lon, and a few decoded fields used by the summary.
    """
    cond_sub, lowest_cond = _condition_subscore(row)
    scour_sub, scour_code = _scour_subscore(row)
    age_sub, age = _age_subscore(row)
    exp_sub, adt = _exposure_subscore(row)
    over_sub, over_info = _overdue_subscore(row)

    subs = {
        "condition": cond_sub,
        "scour": scour_sub,
        "overdue": over_sub,
        "exposure": exp_sub,
        "age": age_sub,
    }

    # Weighted sum, rescaled so the maximum possible score is exactly 100.
    weight_total = sum(WEIGHTS.values())
    raw = sum(subs[k] * WEIGHTS[k] for k in WEIGHTS)
    score = round(100.0 * raw / weight_total, 1)

    # Per-component contribution in score points (for transparency / reasons).
    contrib = {k: 100.0 * subs[k] * WEIGHTS[k] / weight_total for k in WEIGHTS}

    reasons = _build_reasons(contrib, lowest_cond, scour_code, age, adt, over_info)

    return {
        "structure_number": row.get("STRUCTURE_NUMBER_008"),
        "features_desc": row.get("FEATURES_DESC_006A"),
        "facility_carried": row.get("FACILITY_CARRIED_007"),
        "lat": parse_coordinate(row.get("LAT_016"), is_longitude=False),
        "lon": parse_coordinate(row.get("LONG_017"), is_longitude=True),
        "score": score,
        "reasons": reasons,
        "components": {k: round(contrib[k], 1) for k in WEIGHTS},
        "lowest_condition": lowest_cond,
        "scour_code": scour_code,
        "year_built": age if age is None else TODAY.year - age,
        "adt": adt,
        "bridge_condition": row.get("BRIDGE_CONDITION"),
        "over_water": str(row.get("SERVICE_UND_042B") or "") in _OVER_WATER_CODES,
        "underwater": over_info,
    }


def _build_reasons(contrib, lowest_cond, scour_code, age, adt, over_info) -> list[str]:
    """Turn the largest-contributing components into plain-English strings."""
    phrases = {}

    if scour_code and scour_code.isdigit() and int(scour_code) <= 3:
        phrases["scour"] = f"Scour critical (NBI item 113 = {scour_code})"
    elif scour_code == "U":
        phrases["scour"] = "Scour status unknown (item 113 = U)"
    else:
        phrases["scour"] = "Scour rating acceptable"

    if lowest_cond is not None:
        label = "poor" if lowest_cond <= 4 else ("fair" if lowest_cond <= 6 else "good")
        phrases["condition"] = f"Worst condition rating {lowest_cond}/9 ({label})"
    else:
        phrases["condition"] = "No condition ratings available"

    if over_info.get("overdue"):
        mo = over_info.get("months_overdue")
        if mo is None:
            phrases["overdue"] = "Underwater inspection due, no last-date on record"
        else:
            phrases["overdue"] = f"Last underwater date in the 2022 file is overdue by {mo} months"
    elif over_info.get("required"):
        phrases["overdue"] = "Underwater inspection on schedule"
    else:
        phrases["overdue"] = "No underwater inspection required"

    if adt:
        phrases["exposure"] = f"Carries ~{adt:,} vehicles/day"
    else:
        phrases["exposure"] = "Low or unknown traffic"

    if age is not None:
        phrases["age"] = f"{age} years old"
    else:
        phrases["age"] = "Age unknown"

    # Rank components by their score contribution, keep the top 3 that actually
    # contribute something, and fall back to listing drivers if all are ~0.
    ranked = sorted(WEIGHTS, key=lambda k: contrib[k], reverse=True)
    top = [phrases[k] for k in ranked if contrib[k] > 0.5][:3]
    if not top:
        top = ["Low risk across all factors"]
    return top


def score_dataframe(df: pd.DataFrame) -> pd.DataFrame:
    """Score every row; return a DataFrame sorted by descending risk score."""
    records = [score_row(r) for _, r in df.iterrows()]
    out = pd.DataFrame(records)
    return out.sort_values("score", ascending=False).reset_index(drop=True)


# --------------------------------------------------------------------------- #
# NBI -> SNBI mapping
# --------------------------------------------------------------------------- #

def snbi_fields(row) -> dict:
    """
    Map the scour / underwater NBI items used by this tool to their FHWA SNBI
    (Specification for the National Bridge Inventory, 2022) equivalents.

    SNBI reorganises the legacy NBI coding guide into grouped data items with
    new identifiers (e.g. B.IE.* inspection events, B.C.* condition). Where an
    SNBI item id is well established it is given; where the exact id is
    uncertain the 'mapping_note' says so explicitly. We do NOT invent ids.

    Returns a dict keyed by the legacy NBI item, each value a dict with the
    legacy value, a best-effort SNBI item id (or None), and a mapping_note.
    """
    def val(col):
        v = row.get(col)
        return None if v is None else str(v).strip()

    return {
        "SCOUR_CRITICAL_113": {
            "nbi_value": val("SCOUR_CRITICAL_113"),
            "snbi_item": "B.AP.03",
            "snbi_name": "Scour Vulnerability / Scour Condition Rating",
            "mapping_note": (
                "SNBI replaces the single legacy item 113 scour-critical code "
                "with scour-appraisal items in the B.AP (Appraisal) group. The "
                "exact B.AP sub-item id for the scour condition rating is not "
                "certain from memory; verify against the SNBI coding guide "
                "before relying on this id."
            ),
        },
        "UNDWATER_LOOK_SEE_092B": {
            "nbi_value": val("UNDWATER_LOOK_SEE_092B"),
            "snbi_item": None,
            "snbi_name": "Underwater Inspection Required / Frequency",
            "mapping_note": (
                "SNBI moves inspection scheduling into the Inspection (B.IE) "
                "group, splitting the legacy Y+interval code into a separate "
                "'underwater inspection required' flag and an interval/frequency "
                "field. Exact SNBI item ids unknown from memory; do not rely on "
                "a specific id without checking the SNBI guide."
            ),
        },
        "UNDWATER_LAST_DATE_093B": {
            "nbi_value": val("UNDWATER_LAST_DATE_093B"),
            "snbi_item": None,
            "snbi_name": "Date of Last Underwater Inspection",
            "mapping_note": (
                "SNBI records inspection events with dates in the B.IE "
                "inspection group rather than a single MMYY field. The specific "
                "SNBI item id for the last underwater inspection date is not "
                "known from memory; confirm against the SNBI specification."
            ),
        },
        "CHANNEL_COND_061": {
            "nbi_value": val("CHANNEL_COND_061"),
            "snbi_item": None,
            "snbi_name": "Channel / Channel Protection Condition",
            "mapping_note": (
                "Legacy item 061 channel condition maps into the SNBI condition "
                "(B.C) group for channel/waterway. Exact SNBI item id uncertain; "
                "verify before use."
            ),
        },
        "SUBSTRUCTURE_COND_060": {
            "nbi_value": val("SUBSTRUCTURE_COND_060"),
            "snbi_item": None,
            "snbi_name": "Substructure Condition",
            "mapping_note": (
                "Legacy item 060 substructure condition corresponds to a "
                "substructure condition item in the SNBI B.C condition group. "
                "Exact id not known from memory; confirm against the SNBI guide."
            ),
        },
    }


# --------------------------------------------------------------------------- #
# Summary / reporting
# --------------------------------------------------------------------------- #

def build_summary(scored: pd.DataFrame) -> dict:
    """Statewide counts for the --summary command."""
    total = len(scored)
    over_water = int(scored["over_water"].sum())

    scour_critical = scored["scour_code"].apply(
        lambda c: bool(c) and str(c).isdigit() and int(c) <= 3
    )
    n_scour_critical = int(scour_critical.sum())

    poor = scored["bridge_condition"].eq("P")
    n_poor = int(poor.sum())

    required = scored["underwater"].apply(lambda d: bool(d.get("required")))
    overdue = scored["underwater"].apply(lambda d: bool(d.get("overdue")))
    n_required = int(required.sum())
    n_overdue = int(overdue.sum())
    # "Due" here = required and not currently overdue (on the schedule).
    n_due_future = int((required & ~overdue).sum())

    # Daily traffic across scour-critical bridges.
    adt_on_scour = int(
        scored.loc[scour_critical, "adt"].fillna(0).astype(float).sum()
    )

    return {
        "bridges_total": total,
        "over_water": over_water,
        "scour_critical": n_scour_critical,
        "scour_critical_and_poor": int((scour_critical & poor).sum()),
        "poor_condition": n_poor,
        "underwater_inspection_required": n_required,
        "underwater_due_on_schedule": n_due_future,
        "underwater_overdue": n_overdue,
        "daily_traffic_on_scour_critical": adt_on_scour,
    }


def _fmt_table(scored: pd.DataFrame, n: int) -> str:
    """Render the top-n ranked table as aligned plain text."""
    rows = scored.head(n)
    header = f"{'#':>3}  {'score':>5}  {'struct_no':<17}  {'carries / over':<34}  reasons"
    lines = [header, "-" * len(header)]
    for i, (_, r) in enumerate(rows.iterrows(), start=1):
        carries = (r["facility_carried"] or "")[:16]
        over = (r["features_desc"] or "")[:16]
        label = f"{carries} / {over}"[:34]
        reason = "; ".join(r["reasons"][:2])
        lines.append(
            f"{i:>3}  {r['score']:>5.1f}  {str(r['structure_number']):<17}  "
            f"{label:<34}  {reason}"
        )
    return "\n".join(lines)


def _records_for_json(scored: pd.DataFrame) -> list[dict]:
    """All bridges with score, reasons, lat/lon and decoded context for JSON."""
    out = []
    for _, r in scored.iterrows():
        out.append(
            {
                "structure_number": r["structure_number"],
                "features_desc": r["features_desc"],
                "facility_carried": r["facility_carried"],
                "lat": r["lat"],
                "lon": r["lon"],
                "score": r["score"],
                "reasons": r["reasons"],
                "components": r["components"],
                "lowest_condition": r["lowest_condition"],
                "scour_code": r["scour_code"],
                "year_built": r["year_built"],
                "adt": r["adt"],
                "bridge_condition": r["bridge_condition"],
                "over_water": bool(r["over_water"]),
                "underwater": r["underwater"],
            }
        )
    return out


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Risk-based triage of MA bridges for underwater (scour) inspection."
    )
    parser.add_argument("--data", default=DEFAULT_NBI_PATH, help="path to NBI file")
    parser.add_argument("--top", type=int, metavar="N", help="print top-N ranked table")
    parser.add_argument("--json", metavar="OUT", help="write all bridges to a JSON file")
    parser.add_argument("--summary", action="store_true", help="print statewide counts")
    args = parser.parse_args(argv)

    # Default to a small top table if no action was requested.
    if not (args.top or args.json or args.summary):
        args.top = 25

    df = load_nbi(args.data)
    scored = score_dataframe(df)

    if args.summary:
        s = build_summary(scored)
        print("Massachusetts bridges (NBI 2022) -- statewide summary")
        print("-" * 52)
        print(f"  Bridges total                       : {s['bridges_total']:,}")
        print(f"  Over water (SERVICE_UND 5-9)        : {s['over_water']:,}")
        print(f"  Scour critical (item 113 = 0-3)     : {s['scour_critical']:,}")
        print(f"    ...and in Poor condition          : {s['scour_critical_and_poor']:,}")
        print(f"  Poor condition (BRIDGE_CONDITION P) : {s['poor_condition']:,}")
        print(f"  Underwater inspection required      : {s['underwater_inspection_required']:,}")
        print(f"    ...on schedule (not yet due)      : {s['underwater_due_on_schedule']:,}")
        print(f"    ...overdue vs Oct 2026 (2022 dates): {s['underwater_overdue']:,}  (newer dives are not in the 2022 file)")
        print(f"  Daily traffic on scour-critical     : {s['daily_traffic_on_scour_critical']:,} veh/day")
        print()

    if args.top:
        print(f"Top {args.top} bridges by risk score (0-100):")
        print(_fmt_table(scored, args.top))
        print()

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(_records_for_json(scored), fh, indent=2)
        print(f"Wrote {len(scored):,} bridges to {args.json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
