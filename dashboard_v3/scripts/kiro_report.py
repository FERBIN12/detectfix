#!/usr/bin/env python3
"""Generate the Kiro-written inspection report for the dashboard.

Two stages:

  (a) collect a *facts* JSON entirely from already-exported data + the bags +
      the NBI file. No numbers are invented; every value
      traces back to the bag, the maps, the ground-truth defect list or FHWA
      NBI 2022. The facts cover:
        - this mission's statistics (from web/data/stats.json / series.json);
        - a comparison with the previous drone-only mission (PREV): what is new
          this run (underwater coverage, sonar, ROV tasks) with PREV's own
          message/frame/lidar counts read from its metadata;
        - a statewide comparison from the NBI file: the dam crossing's two
          bridges (B160134EM Craigie drawbridge, B160134ET 10-span dam) vs.
          Massachusetts (condition ratings, scour code, ADT), and how many MA
          bridges are Poor / scour critical / need underwater inspection (plain
          FHWA NBI 2022 counts — no statewide ranking);
        - a cost comparison (divers only vs. robot-first + targeted dive);
        - model detections vs. ground truth (which GT defects were near a
          detection, honestly, including the misses).

  (b) call `kiro-cli chat --no-interactive --trust-all-tools "<prompt>"` with
      the facts JSON embedded, asking Kiro to write a concise professional
      Markdown inspection report, saved to web/data/kiro_report.md. The facts
      JSON is saved alongside as web/data/kiro_report_facts.json.

Run once after export_web.py so the dashboard can render the report panel.

    python scripts/kiro_report.py --bag <BAG> --prev <PREV_BAG>
"""
import argparse
import datetime as _dt
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent          # repo root (detectfix/)
WEB_DATA = HERE / "web" / "data"

HOME = Path.home()
DEFAULT_BAG = HOME / "hackachusetts" / "bags" / "run_20261024_134808"
DEFAULT_PREV = HOME / "hackachusetts" / "bags" / "run_20261024_123507"
NBI_PATH = HOME / "hackachusetts" / "data" / "nbi" / "MA22.txt"
TRIAGE_PY = HOME / "hackachusetts" / "ai" / "bridge_triage.py"
TRIAGE_VENV = HOME / "hackachusetts" / ".venv" / "bin" / "python"
KIRO_CLI = os.environ.get("KIRO_CLI", str(HOME / ".local" / "bin" / "kiro-cli"))

# The two NBI structures at the Charles River Dam crossing.
DAM_BRIDGES = {
    "B160134EMDOTNBI": "Craigie drawbridge (steel bascule, Route 28, built 1910)",
    "B160134ETDOTNBI": "Charles River Dam 10-span bridge (built 1906)",
}

# Cost figures (documented, not invented).
COSTS = {
    "divers_only_usd_per_bridge_per_dive_team_day": [6800, 10800],
    "divers_only_source": "FHWA 2012 dollars (dive-team day rate)",
    "robot_first_then_targeted_dive_usd": [5300, 7600],
    "robot_first_source": "team estimate (robot screening + one targeted dive)",
}


# --------------------------------------------------------------------------- #
# metadata helpers (no PyYAML dependency)
# --------------------------------------------------------------------------- #
def parse_metadata(bag_dir: Path) -> dict:
    """Total messages / duration / per-topic counts from metadata.yaml."""
    meta = bag_dir / "metadata.yaml"
    info = {"total_messages": None, "duration_s": None, "topics": {}}
    if not meta.exists():
        return info
    text = meta.read_text(encoding="utf-8", errors="replace")
    mm = re.search(r"message_count:\s*(\d+)", text)
    if mm:
        info["total_messages"] = int(mm.group(1))
    md = re.search(r"duration:\s*\n\s*nanoseconds:\s*(\d+)", text)
    if md:
        info["duration_s"] = round(int(md.group(1)) / 1e9, 1)
    names = re.findall(r"name:\s*(/\S+)", text)
    counts = re.findall(r"^\s*message_count:\s*(\d+)\s*$", text, re.MULTILINE)
    per_topic = counts[1:] if len(counts) > len(names) else counts
    for name, c in zip(names, per_topic):
        info["topics"][name] = int(c)
    return info


# --------------------------------------------------------------------------- #
# NBI + triage extraction (runs in the triage venv, which has pandas)
# --------------------------------------------------------------------------- #
_NBI_EXTRACT = r"""
import json, sys
sys.argv = ["bridge_triage"]
sys.path.insert(0, {ai!r})
import bridge_triage as bt

df = bt.load_nbi({nbi!r})
scored = bt.score_dataframe(df)
summary = bt.build_summary(scored)

wanted = {wanted!r}
rows = []
for _, r in scored.iterrows():
    sn = r["structure_number"]
    if sn in wanted:
        rows.append({{
            "structure_number": sn,
            "lowest_condition": r["lowest_condition"],
            "scour_code": r["scour_code"],
            "year_built": r["year_built"],
            "adt": r["adt"],
            "bridge_condition": r["bridge_condition"],
            "over_water": bool(r["over_water"]),
            "facility_carried": (r["facility_carried"] or "").strip(),
            "features_desc": (r["features_desc"] or "").strip(),
            "underwater": r["underwater"],
        }})

# also pull raw condition fields (deck/super/sub/channel) for the two bridges
raw = {{}}
for _, r in df.iterrows():
    sn = r.get("STRUCTURE_NUMBER_008")
    if sn in wanted:
        raw[sn] = {{
            "deck": r.get("DECK_COND_058"),
            "superstructure": r.get("SUPERSTRUCTURE_COND_059"),
            "substructure": r.get("SUBSTRUCTURE_COND_060"),
            "channel": r.get("CHANNEL_COND_061"),
            "scour_113": r.get("SCOUR_CRITICAL_113"),
            "adt": r.get("ADT_029"),
        }}

print(json.dumps({{"summary": summary, "bridges": rows, "raw": raw,
                   "total": int(len(scored))}}))
"""


def extract_nbi_facts() -> dict:
    """Run bridge_triage in its own venv to get the two bridges + statewide."""
    code = _NBI_EXTRACT.format(
        ai=str(TRIAGE_PY.parent), nbi=str(NBI_PATH),
        wanted=list(DAM_BRIDGES.keys()),
    )
    if not TRIAGE_VENV.exists():
        return {"error": f"triage venv not found at {TRIAGE_VENV}"}
    proc = subprocess.run([str(TRIAGE_VENV), "-c", code],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        return {"error": proc.stderr[-800:]}
    try:
        return json.loads(proc.stdout.strip().splitlines()[-1])
    except Exception as exc:  # pragma: no cover
        return {"error": f"parse failed: {exc}", "stdout": proc.stdout[-400:]}


# --------------------------------------------------------------------------- #
# detections vs. ground truth (honest proximity match)
# --------------------------------------------------------------------------- #
def detections_vs_gt(result: dict, scene: dict, radius_m: float = 8.0) -> dict:
    """Match each model detection to the nearest GT defect within radius_m.

    Reports which GT defects were near a detection, the misses, and the exact
    nearest distance for each detection. Detections are localized at the drone
    camera pose, so distances of several metres to the true defect are expected
    and reported honestly rather than hidden.
    """
    gt = scene.get("defects", [])
    dets = scene.get("detections", [])
    matched_gt, matches = set(), []
    for d in dets:
        dp = d.get("pos")
        if not dp:
            continue
        best, bestd = None, 1e9
        for g in gt:
            dist = math.dist(dp, g["pos"])
            if dist < bestd:
                bestd, best = dist, g
        if best and bestd <= radius_m:
            matched_gt.add(best["id"])
            matches.append({"detection": d.get("id"), "det_class": d.get("class"),
                            "gt_id": best["id"], "gt_type": best["type"],
                            "distance_m": round(bestd, 2),
                            "underwater": best["underwater"]})
        else:
            matches.append({"detection": d.get("id"), "det_class": d.get("class"),
                            "gt_id": None, "nearest_m": round(bestd, 2)})
    caught = sorted(matched_gt)
    missed = [g["id"] for g in gt if g["id"] not in matched_gt]
    missed_above = [g["id"] for g in gt if g["id"] not in matched_gt and not g["underwater"]]
    missed_below = [g["id"] for g in gt if g["id"] not in matched_gt and g["underwater"]]
    return {
        "n_detections": len(dets),
        "n_gt": len(gt),
        "caught_gt_ids": caught,
        "missed_gt_ids": missed,
        "missed_above_water": missed_above,
        "missed_below_water": missed_below,
        "matches": matches,
        "match_radius_m": radius_m,
        "note": ("Detections run only on the drone RGB camera above the water line; "
                 "the 4 underwater defects are surfaced from the cooperation log / "
                 "ground truth, not a trained sonar model."),
    }


# --------------------------------------------------------------------------- #
# cooperation: waterline handoffs vs. underwater survey coverage
# --------------------------------------------------------------------------- #
def handoff_vs_survey(scene: dict, coop: dict) -> dict:
    """Separate the drone→ROV *handoffs* from the ROV's *survey* coverage.

    Two distinct things, kept factually separate:

      * HANDOFFS — the drone flagged 4 defects that sit just ABOVE the water
        line (z ≈ 0.8–1.5 m). For each, the ROV was tasked (coop_log.txt /
        scene dive_lines) to dive and inspect the submerged wall directly
        BELOW that flag. These are the recommended dive spots.

      * SURVEY COVERAGE — the 4 genuinely UNDERWATER ground-truth defects
        (D10 scour, D11 missing_block, D12 crack, D13 undermining) were NOT
        handoffs. They lie along the ROV's planned survey route and were
        covered by its multibeam sonar survey. For each we report its distance
        to the nearest point on the ROV ground-truth (truth) path from the bag.
    """
    defects = scene.get("defects", [])
    by_id = {d["id"]: d for d in defects}
    truth_path = scene.get("rov_truth_path", []) or []

    def nearest_truth_dist(pos):
        if not truth_path or not pos:
            return None
        return round(min(math.dist(pos, p) for p in truth_path), 2)

    # Handoffs come from the dive lines the exporter parsed out of coop_log.txt.
    dive = (coop.get("log") or {})
    lines = dive.get("lines", [])
    done = set(dive.get("done", []))
    handoffs = []
    for ln in lines:
        did = ln.get("id")
        d = by_id.get(did, {})
        top = ln.get("top") or d.get("pos")
        handoffs.append({
            "defect_id": did,
            "type": ln.get("type") or d.get("type"),
            "element": d.get("element", ""),
            "flag_z_m": round(top[2], 2) if top else None,
            "dive_point": ln.get("bottom"),
            "inspected": did in done,
            "note": "Above the water line; ROV tasked to dive and inspect the "
                    "submerged wall directly below this flag.",
        })

    # Underwater GT defects covered by the sonar survey (not handoffs).
    survey = []
    for d in defects:
        if not d.get("underwater"):
            continue
        survey.append({
            "defect_id": d["id"],
            "type": d.get("type"),
            "element": d.get("element", ""),
            "pos": d.get("pos"),
            "depth_m": round(abs(d["pos"][2]), 2) if d.get("pos") else None,
            "dist_to_nearest_rov_truth_point_m": nearest_truth_dist(d.get("pos")),
            "note": "Underwater; NOT a drone handoff. Lies on the ROV survey "
                    "route and was covered by the multibeam sonar survey.",
        })

    return {
        "handoffs": handoffs,
        "n_handoffs": len(handoffs),
        "survey_covered_underwater": survey,
        "n_survey_covered": len(survey),
        "truth_path_points": len(truth_path),
        "explanation": (
            "The 4 drone-flagged waterline defects (D03, D01, D07, D05; z≈0.8–1.5 m) "
            "are handoffs: the ROV dived to inspect the submerged wall directly below "
            "each one — these are the recommended dive spots. The 4 underwater defects "
            "(D10 scour, D11 missing_block, D12 crack, D13 undermining) are NOT handoffs; "
            "they lie along the ROV's survey route and were covered by its multibeam "
            "sonar survey. Distances above are each defect's range to the nearest point "
            "on the ROV ground-truth path from the bag."
        ),
    }


# --------------------------------------------------------------------------- #
# facts assembly
# --------------------------------------------------------------------------- #
def load_json(path: Path, default=None):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return default


def build_facts(bag_dir: Path, prev_dir: Path) -> dict:
    stats = load_json(WEB_DATA / "stats.json", {})
    series = load_json(WEB_DATA / "series.json", {})
    kpis = load_json(WEB_DATA / "kpis.json", {})
    scene = load_json(WEB_DATA / "scene.json", {})
    result = load_json(WEB_DATA / "result.json", {})
    coop = load_json(WEB_DATA / "coop.json", {})
    frames = load_json(WEB_DATA / "frames.json", {})

    this_meta = parse_metadata(bag_dir)
    prev_meta = parse_metadata(prev_dir)

    # mission stats as a flat label->value map for easy prompting.
    mission = {}
    for row in (stats.get("rows") or []):
        key = f"{row['group']} :: {row['label'].strip()}"
        mission[key] = {"value": row["value"], "unit": row["unit"]}

    # previous mission comparison -- PREV is drone-only.
    prev_topics = prev_meta.get("topics", {})
    this_topics = this_meta.get("topics", {})
    prev_cmp = {
        "prev_mission_id": prev_dir.name,
        "prev_total_messages": prev_meta.get("total_messages"),
        "prev_duration_s": prev_meta.get("duration_s"),
        "prev_lidar_msgs": prev_topics.get("/lidar/points"),
        "prev_rgb_frames": prev_topics.get("/camera/rgb"),
        "prev_thermal_frames": prev_topics.get("/camera/thermal"),
        "prev_has_rov": any(t.startswith("/rov/") for t in prev_topics),
        "prev_has_sonar": "/rov/sonar/points" in prev_topics,
        "prev_has_coop": "/coop/tasks" in prev_topics,
        "this_mission_id": bag_dir.name,
        "this_total_messages": this_meta.get("total_messages"),
        "this_duration_s": this_meta.get("duration_s"),
        "this_lidar_msgs": this_topics.get("/lidar/points"),
        "this_sonar_point_msgs": this_topics.get("/rov/sonar/points"),
        "this_rov_tasks": len((coop.get("log") or {}).get("done", [])),
        "new_this_run": [
            "underwater survey with the BlueROV2 (ArduSub) — absent in the previous run",
            "multibeam sonar map of the canal bed and submerged walls",
            "DVL dead-reckoned navigation with ground-truth error measurement",
            "drone→ROV cooperation (waterline flags handed to the ROV to dive)",
        ],
    }

    nbi = extract_nbi_facts()
    det_gt = detections_vs_gt(result, scene)
    cooperation = handoff_vs_survey(scene, coop)

    facts = {
        "generated_at": _dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "mission_id": scene.get("mission_id", bag_dir.name),
        "bridge": scene.get("bridge", ""),
        "site": "Craigie Bridge / Charles River Dam lock canal, Boston, MA",
        "mission_statistics": mission,
        "kpis": kpis,
        "previous_mission_comparison": prev_cmp,
        "statewide_nbi": nbi,
        "dam_bridge_labels": DAM_BRIDGES,
        "cost_comparison": COSTS,
        "detections_vs_ground_truth": det_gt,
        "cooperation_handoffs_vs_survey": cooperation,
        "limitations": [
            "This is a simulated mission (Isaac Sim + ArduPilot SITL), not a live bridge.",
            "The thermal feed is simulated LWIR.",
            "Waterline defect flags use ground truth in this demo (no live operator).",
            "No trained sonar defect model is run; the 4 underwater defects are ground "
            "truth covered by the ROV's sonar survey route, not model detections.",
            "NBI 'overdue' is measured against the 2022 file; dives newer than 2022 are not in it.",
        ],
    }
    return facts


# --------------------------------------------------------------------------- #
# Kiro call
# --------------------------------------------------------------------------- #
PROMPT_TEMPLATE = """You are a professional bridge-inspection engineer writing a concise field \
inspection report in GitHub-flavoured Markdown. Use ONLY the facts in the JSON below; \
do not invent numbers. Be honest about limitations. Keep it tight (roughly 450-650 words).

CRITICAL — state the drone↔ROV cooperation exactly (see cooperation_handoffs_vs_survey):
- The drone flagged 4 WATERLINE defects just above the water line (D03 efflorescence,
  D01 spall, D07 efflorescence, D05 crack; z≈0.8–1.5 m). For EACH of these the ROV was
  tasked to dive and inspect the submerged wall DIRECTLY BELOW the flag. These handoffs
  are the recommended dive spots.
- The 4 UNDERWATER defects (D10 scour, D11 missing_block, D12 crack, D13 undermining)
  were NOT handoffs. They lie along the ROV's survey route and were covered by its
  multibeam sonar survey. Give each one's distance to the nearest ROV ground-truth path
  point (dist_to_nearest_rov_truth_point_m). Do NOT describe these as drone handoffs.

Write these sections with Markdown headings:
# Craigie Bridge / Charles River Dam — Inspection Report
## Summary  (2-3 sentences: what was flown/dived, the headline findings)
## Findings by element
### Above the water line (drone)  (bullet the key GT defects caught/missed, elements; note the 4 waterline flags handed to the ROV)
### Below the water line (BlueROV2)  (scour / submerged wall findings covered by the sonar survey with each defect's distance to the ROV route; DVL nav accuracy)
## Comparison with the previous mission  (what is new this run vs the drone-only run)
## Statewide context (FHWA NBI 2022)  (the two dam-crossing bridges' condition ratings, scour code and ADT vs Massachusetts: Poor / scour critical / underwater-required counts — plain NBI counts, do NOT rank the bridges statewide; a short cost comparison divers vs robot-first)
## Recommended next dive spots  (the 4 waterline handoffs — dive the submerged wall directly below D03, D01, D07, D05 — consistent with the cooperation handoffs)
## Limitations  (bullet the listed limitations plainly)

Return ONLY the Markdown report, no preamble, no code fences around the whole thing.

FACTS JSON:
{facts}
"""


def call_kiro(facts: dict, out_md: Path) -> bool:
    prompt = PROMPT_TEMPLATE.format(facts=json.dumps(facts, indent=1))
    if not Path(KIRO_CLI).exists():
        print(f"  ERROR kiro-cli not found at {KIRO_CLI}", file=sys.stderr)
        return False
    print(f"  calling kiro-cli ({KIRO_CLI}) — this may take a minute …")
    try:
        proc = subprocess.run(
            [KIRO_CLI, "chat", "--no-interactive", "--trust-all-tools", prompt],
            capture_output=True, text=True, timeout=600,
        )
    except subprocess.TimeoutExpired:
        print("  ERROR kiro-cli timed out", file=sys.stderr)
        return False
    out = proc.stdout.strip()
    if proc.returncode != 0 or not out:
        print(f"  ERROR kiro-cli rc={proc.returncode}\n{proc.stderr[-800:]}", file=sys.stderr)
        return False
    md = _clean_markdown(out)
    out_md.write_text(md, encoding="utf-8")
    print(f"  wrote {out_md} ({len(md)} chars)")
    return True


def _clean_markdown(text: str) -> str:
    """Strip any wrapping ``` fences and leading chatter before the first #."""
    t = text.strip()
    # drop a leading fenced block wrapper if the whole thing is fenced
    if t.startswith("```"):
        t = re.sub(r"^```[a-zA-Z]*\n", "", t)
        t = re.sub(r"\n```\s*$", "", t)
    # trim anything before the first markdown heading, if present
    idx = t.find("# ")
    if idx > 0:
        t = t[idx:]
    return t.strip() + "\n"


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description="Generate the Kiro inspection report")
    ap.add_argument("--bag", default=str(DEFAULT_BAG))
    ap.add_argument("--prev", default=str(DEFAULT_PREV))
    ap.add_argument("--facts-only", action="store_true",
                    help="write the facts JSON but skip the kiro-cli call")
    args = ap.parse_args()

    bag_dir = Path(args.bag)
    prev_dir = Path(args.prev)
    WEB_DATA.mkdir(parents=True, exist_ok=True)

    print("collecting facts …")
    facts = build_facts(bag_dir, prev_dir)
    facts_path = WEB_DATA / "kiro_report_facts.json"
    facts_path.write_text(json.dumps(facts, indent=1), encoding="utf-8")
    print(f"  facts -> {facts_path}")
    nbi = facts.get("statewide_nbi", {})
    if "error" in nbi:
        print(f"  WARN NBI extraction error: {nbi['error'][:200]}")
    else:
        conds = {b['structure_number']: b['lowest_condition']
                 for b in nbi.get('bridges', [])}
        print(f"  NBI: {nbi.get('total')} MA bridges; dam-bridge conditions {conds}")
    dg = facts["detections_vs_ground_truth"]
    print(f"  detections {dg['n_detections']} vs GT {dg['n_gt']}: "
          f"caught {dg['caught_gt_ids']}")

    if args.facts_only:
        print("facts-only: skipping kiro-cli")
        return 0

    print("generating report with Kiro …")
    ok = call_kiro(facts, WEB_DATA / "kiro_report.md")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
