"""Human sign-off gate for the inspection report.

    draft --submit--> under_review --approve--> approved
                           |  ^
                    reject |  | resubmit (after revising)
                           v  |
                         rejected

Code enforces the gate: only the transitions above are allowed; approving or
rejecting needs a named reviewer; and every approval is pinned to a hash of
the exact report it approved. If the pipeline is re-run and the report or its
detections change, the stored state no longer matches and falls back to
"draft" -- an approval can never carry over to a report nobody reviewed.

The AI never changes this state. Standard library only.
"""
from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path

STATES = ("draft", "under_review", "approved", "rejected")
# action -> (allowed from-states, to-state, needs a reviewer name)
ACTIONS = {
    "submit":   (("draft",), "under_review", False),
    "approve":  (("under_review",), "approved", True),
    "reject":   (("under_review",), "rejected", True),
    "resubmit": (("rejected",), "under_review", False),
}


class ReviewError(ValueError):
    pass


def report_hash(result: dict) -> str:
    """Hash of what a reviewer signs: the narrative and every detection's id,
    class, severity and element."""
    rep = result.get("report") or {}
    dets = sorted((d.get("detection_id"), d.get("class"), d.get("severity"),
                   ((d.get("location") or {}).get("structure") or {}).get("element_id"))
                  for d in result.get("detections", []))
    blob = json.dumps({"narrative": rep.get("narrative", ""), "detections": dets}, sort_keys=True)
    return hashlib.sha256(blob.encode()).hexdigest()[:16]


def _path(store_dir: Path, mission_id: str) -> Path:
    safe = "".join(c for c in mission_id if c.isalnum() or c in "-_") or "mission"
    return Path(store_dir) / f"{safe}.json"


def load(store_dir: Path, result: dict) -> dict:
    """Current review record for this result (draft if none, or if the stored
    record was for a different version of the report)."""
    h = report_hash(result)
    p = _path(store_dir, result.get("mission_id", ""))
    if p.exists():
        rec = json.loads(p.read_text(encoding="utf-8"))
        if rec.get("report_hash") == h:
            return rec
        stale = rec.get("state")
    else:
        stale = None
    rec = {"mission_id": result.get("mission_id", ""), "report_hash": h, "state": "draft", "history": []}
    if stale and stale != "draft":
        rec["note"] = f"previous review ({stale}) was for a different report version; reset to draft"
    return rec


def apply(store_dir: Path, result: dict, action: str, reviewer: str = "", note: str = "",
          expected_hash: str = "") -> dict:
    """Apply one transition and persist it. Raises ReviewError if not allowed."""
    if action not in ACTIONS:
        raise ReviewError(f"unknown action {action!r}")
    rec = load(store_dir, result)
    if expected_hash and expected_hash != rec["report_hash"]:
        raise ReviewError("the report changed since you loaded it; reload before reviewing")
    allowed, to, needs_name = ACTIONS[action]
    if rec["state"] not in allowed:
        raise ReviewError(f"cannot {action} a report that is {rec['state']}")
    reviewer = reviewer.strip()[:80]
    if needs_name and not reviewer:
        raise ReviewError(f"{action} needs a reviewer name")
    rec["history"].append({"action": action, "from": rec["state"], "to": to, "reviewer": reviewer,
                           "note": note.strip()[:500],
                           "at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())})
    rec["state"] = to
    rec.pop("note", None)
    if to in ("approved", "rejected"):
        rec["reviewer"] = reviewer
    p = _path(store_dir, rec["mission_id"])
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(rec, indent=2), encoding="utf-8")
    return rec
