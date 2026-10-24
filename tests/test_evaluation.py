"""Unit tests for pipeline/evaluation.py and the no-.mcap frame fallback.
Standard library only. Run: python -m unittest discover -s tests
"""
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from pipeline import evaluation as ev  # noqa: E402
from pipeline.evaluation import GTDefect, Prediction  # noqa: E402

SAMPLE = ROOT / "sample_data" / "run_20261024_123507"


def gt(gid, cls="crack", element="wall", pos=(0.0, 0.0, 0.0), sev=2):
    return GTDefect(id=gid, cls=cls, severity=sev, element=element, position=tuple(pos))


def pred(pid, cls="crack", element="wall", pos=(0.0, 0.0, 0.0), conf=0.9):
    return Prediction(id=pid, cls=cls, confidence=conf, element=element,
                      position=None if pos is None else tuple(pos))


class TestLoadGroundTruth(unittest.TestCase):
    def test_sample_ground_truth_loads_all_nine(self):
        gts, skipped = ev.load_ground_truth(SAMPLE / "defects_gt.json")
        self.assertEqual(len(gts), 9)
        self.assertEqual(skipped, [])
        self.assertEqual({g.cls for g in gts}, {"spall", "crack", "efflorescence", "rust_streak"})

    def test_unknown_type_is_skipped_not_fatal(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "gt.json"
            p.write_text(json.dumps({"defects": [
                {"id": "A", "type": "crack", "position": [0, 0, 0]},
                {"id": "B", "type": "graffiti", "position": [1, 1, 1]}]}), encoding="utf-8")
            gts, skipped = ev.load_ground_truth(p)
        self.assertEqual([g.id for g in gts], ["A"])
        self.assertEqual(skipped, ["B"])

    def test_missing_and_invalid_files_name_the_path(self):
        with self.assertRaisesRegex(FileNotFoundError, "nope.json"):
            ev.load_ground_truth("nope.json")
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.json"
            p.write_text("{not json", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "bad.json"):
                ev.load_ground_truth(p)


class TestMatch(unittest.TestCase):
    def test_radius_gate(self):
        self.assertEqual(ev.match([pred("p", pos=(3, 0, 0))], [gt("g")], radius_m=2.0).pairs, [])
        self.assertEqual(len(ev.match([pred("p", pos=(3, 0, 0))], [gt("g")], radius_m=3.0).pairs), 1)

    def test_duplicate_predictions_do_not_double_count(self):
        m = ev.match([pred("p1", pos=(0.1, 0, 0)), pred("p2", pos=(0.2, 0, 0))], [gt("g")])
        self.assertEqual([(p.pred_id, p.gt_id) for p in m.pairs], [("p1", "g")])
        self.assertEqual(m.false_positives, ["p2"])

    def test_element_gate(self):
        p = pred("p", element="wall_W")
        self.assertEqual(ev.match([p], [gt("g", element="wall_E")]).pairs, [])
        self.assertEqual(len(ev.match([p], [gt("g", element="wall_E")], element_gate=False).pairs), 1)
        self.assertEqual(len(ev.match([pred("p", element="")], [gt("g", element="wall_E")]).pairs), 1)

    def test_closest_first_and_order_independent(self):
        preds = [pred("far", pos=(2, 0, 0)), pred("near", pos=(1, 0, 0))]
        a = ev.match(preds, [gt("g")])
        b = ev.match(list(reversed(preds)), [gt("g")])
        self.assertEqual(a.pairs, b.pairs)
        self.assertEqual(a.pairs[0].pred_id, "near")

    def test_class_aware_uses_families(self):
        g = gt("g", cls="rust_streak")
        self.assertEqual(len(ev.match([pred("p", cls="corrosion_stain")], [g], class_aware=True).pairs), 1)
        self.assertEqual(ev.match([pred("p", cls="spall")], [g], class_aware=True).pairs, [])
        self.assertEqual(len(ev.match([pred("p", cls="spall")], [g], class_aware=False).pairs), 1)

    def test_prediction_without_position_is_unlocatable_fp(self):
        m = ev.match([pred("p", pos=None)], [gt("g")])
        self.assertEqual(m.unlocatable, ["p"])
        self.assertEqual(m.false_positives, ["p"])
        self.assertEqual(ev.score(m, 1, 1)["fp"], 1)


class TestScore(unittest.TestCase):
    def test_counts_and_ratios(self):
        preds = [pred("p1"), pred("p2", pos=(50, 0, 0))]
        gts = [gt("g1"), gt("g2", pos=(0, 30, 0))]
        s = ev.score(ev.match(preds, gts), len(preds), len(gts))
        self.assertEqual((s["tp"], s["fp"], s["fn"]), (1, 1, 1))
        self.assertEqual((s["precision"], s["recall"], s["f1"]), (0.5, 0.5, 0.5))

    def test_zero_denominators_give_zero(self):
        self.assertEqual(ev.score(ev.match([], []), 0, 0),
                         {"tp": 0, "fp": 0, "fn": 0, "precision": 0.0, "recall": 0.0, "f1": 0.0})

    def test_evaluate_has_both_modes_and_traces(self):
        out = ev.evaluate([pred("p", cls="spall")], [gt("g", cls="crack")])
        self.assertEqual(out["class_agnostic"]["tp"], 1)
        self.assertEqual(out["class_aware"]["tp"], 0)
        self.assertEqual(out["class_agnostic"]["pairs"][0]["gt_id"], "g")
        self.assertEqual(out["class_aware"]["missed"], ["g"])
        self.assertEqual(out["per_class_recall"]["crack"], {"gt": 1, "found": 0, "recall": 0.0})

    def test_sweep_filters_by_confidence(self):
        preds = [pred("lo", conf=0.2, pos=(0, 0, 0)), pred("hi", conf=0.8, pos=(0, 20, 0))]
        gts = [gt("g1"), gt("g2", pos=(0, 20, 0))]
        self.assertEqual([r["tp"] for r in ev.sweep(preds, gts, thresholds=[0.1, 0.5])], [2, 1])


class TestEvalCli(unittest.TestCase):
    def test_cli_round_trip_and_determinism(self):
        result = {"mission_id": "M", "detections": [
            {"detection_id": "D-000000000001", "class": "crack", "confidence": 0.9,
             "world_pos": [-20.4, -17.3, 2.3],
             "location": {"structure": {"element_id": "canal_wall_E"}}}]}
        with tempfile.TemporaryDirectory() as d:
            rp = Path(d) / "r.json"
            rp.write_text(json.dumps(result), encoding="utf-8")
            outs = []
            for i in range(2):
                op = Path(d) / f"e{i}.json"
                proc = subprocess.run([sys.executable, str(ROOT / "eval_bag.py"), "--result", str(rp),
                                       "--bag", str(SAMPLE), "--out", str(op)],
                                      capture_output=True, text=True)
                self.assertEqual(proc.returncode, 0, proc.stderr)
                outs.append(json.loads(op.read_text(encoding="utf-8")))
        self.assertEqual(outs[0], outs[1])
        self.assertEqual(outs[0]["class_aware"]["pairs"][0]["gt_id"], "D02")

    def test_cli_missing_gt_exits_2(self):
        proc = subprocess.run([sys.executable, str(ROOT / "eval_bag.py"), "--result", "x.json",
                               "--gt", "does_not_exist.json"], capture_output=True, text=True)
        self.assertEqual(proc.returncode, 2)
        self.assertIn("does_not_exist.json", proc.stderr)


class TestFrameFallback(unittest.TestCase):
    def test_decode_bag_without_mcap_uses_sidecars(self):
        from pipeline import rosbag
        captures, tl = rosbag.decode_bag(str(SAMPLE))
        self.assertEqual(sum(1 for c in captures if c.kind == "rgb"), 36)
        self.assertGreater(len(tl), 1000)

    def test_sample_files_fit_in_git(self):
        big = [p for p in SAMPLE.rglob("*") if p.is_file() and p.stat().st_size > 50 * 1024 * 1024]
        self.assertEqual(big, [])


if __name__ == "__main__":
    unittest.main()


# --- drone + ROV mission support ---------------------------------------------

def _write_rov_extract(bag: Path, vis=(0.01, 0.2)):
    """A tiny synthetic rov/ extract: 2 sonar pings, 2 camera frames, status."""
    import csv as _csv
    rov = bag / "rov"
    rov.mkdir(parents=True)
    rows = {
        "telemetry.csv": (["t", "x", "y", "z", "tx", "ty", "tz", "drift_m"],
                          [[100, 0, 0, -2, 0, 0, -2, 0.1], [110, 5, 0, -2, 5, 0.5, -2, 0.5]]),
        "status.csv": (["t", "mode", "area", "depth_m", "dvl_alt_m", "dvl_lock", "nav_err_m"],
                       [[100, "GUIDED", "wall_E_submerged", 2.0, 1.5, True, 0.1],
                        [110, "GUIDED", "transit", 2.0, 1.5, False, 0.2]]),
        "sonar_index.csv": (["t", "seq", "x", "y", "z"], [[100, 0, 0, 0, -2], [110, 1, 5, 0, -2]]),
        "camera_index.csv": (["t", "seq", "x", "y", "z", "visibility"],
                             [[100.2, 0, 0, 0, -2, vis[0]], [110.2, 1, 5, 0, -2, vis[1]]]),
        "coop_tasks.csv": (["t", "defect_id", "text"], [[99, "D01", "drone saw D01 spall -> ROV diving"]]),
    }
    for name, (head, data) in rows.items():
        with (rov / name).open("w", newline="", encoding="utf-8") as fh:
            w = _csv.writer(fh)
            w.writerow(head)
            w.writerows(data)
    (bag / "defects_gt.json").write_text(json.dumps({"defects": [
        {"id": "U1", "type": "scour", "severity": 3, "element": "toe", "position": [0.5, 0, -2.5]},
        {"id": "U2", "type": "undermining", "severity": 2, "element": "footing", "position": [5, 0.5, -2]},
        {"id": "A1", "type": "crack", "severity": 2, "element": "wall", "position": [0, 0, 2]},
        {"id": "FAR", "type": "missing_block", "severity": 3, "element": "x", "position": [90, 0, -1]},
    ]}), encoding="utf-8")


class TestRov(unittest.TestCase):
    def test_waterline_splits_platforms(self):
        from pipeline import rosbag
        self.assertEqual(rosbag.gt_platform([0, 0, -0.1]), "rov")
        self.assertEqual(rosbag.gt_platform([0, 0, 0.0]), "drone")
        self.assertEqual(rosbag.gt_platform(None), "drone")

    def test_underwater_classes_validate_on_rov_only(self):
        from pipeline.schema import make_detection
        for cls in ("missing_block", "undermining", "crack", "scour"):
            make_detection("M", "rov", cls, 0.8)
        with self.assertRaises(ValueError):
            make_detection("M", "drone", "missing_block", 0.8)

    def test_missing_block_and_undermining_are_poor_on_presence(self):
        from pipeline import severity
        from pipeline.schema import make_detection
        for cls in ("missing_block", "undermining"):
            self.assertEqual(severity.classify(make_detection("M", "rov", cls, 0.9)), "poor")

    def test_gt_detections_and_clarity_gate(self):
        from pipeline import indexing, rov
        with tempfile.TemporaryDirectory() as d:
            bag = Path(d)
            _write_rov_extract(bag, vis=(0.01, 0.2))
            caps = rov.load_captures(str(bag))
            self.assertEqual(sorted(c.kind for c in caps), ["rov_camera", "rov_camera", "sonar", "sonar"])
            dets = rov.gt_detections(str(bag), "M", [c for c in caps if c.kind == "sonar"])
            # crack above water and the defect 90 m from any ping are not invented
            self.assertEqual(sorted(x.cls for x in dets), ["scour", "undermining"])
            self.assertTrue(all(x.platform == "rov" for x in dets))
            self.assertTrue(all(x.model_version.startswith("ground-truth") for x in dets))
            idx = indexing.build_index(caps, max_dt_s=2.0, max_pose_m=5.0)
            rov.confirm_with_camera(idx, dets)
            by = {x.cls: x for x in dets}
            self.assertFalse(by["scour"].confirmed)        # frame at visibility 0.01: murk
            self.assertTrue(by["undermining"].confirmed)   # frame at visibility 0.2: clear
            self.assertIn("rov_camera:1", by["undermining"].evidence_frame_ids)
            self.assertEqual(by["scour"].location.structure.elevation_or_depth, -2.5)

    def test_rov_health_is_real_when_extract_exists(self):
        from pipeline import health
        with tempfile.TemporaryDirectory() as d:
            self.assertTrue(health.build_health(d)["rov"]["is_filler"])
            _write_rov_extract(Path(d))
            h = health.build_health(d)
            self.assertTrue(h["has_real_rov"])
            r = h["rov"]
            self.assertFalse(r["is_filler"])
            self.assertEqual(r["nav"]["dvl_lock_pct"], 50.0)
            self.assertEqual(r["nav"]["max_drift_m"], 0.5)
            self.assertEqual(r["dives"], 1)
            self.assertEqual(r["sonar"]["frames"], 2)

    def test_status_line_parsing_includes_coop_tasks(self):
        sys.path.insert(0, str(ROOT))
        import extract_rov
        m = extract_rov.STATUS_RE.match(
            "ArduSub GUIDED task D03: below efflorescence  depth 2.5 m  DVL alt 1.55 m lock True  nav err 0.16 m")
        self.assertIsNotNone(m)
        self.assertEqual((m["depth"], m["lock"], m["err"]), ("2.5", "True", "0.16"))
        m = extract_rov.STATUS_RE.match(
            "ArduSub GUIDED wall_E_submerged  depth 2.4 m  DVL alt 1.46 m lock True  nav err 0.09 m")
        self.assertEqual(m["area"].strip(), "wall_E_submerged")


class TestElementAssignment(unittest.TestCase):
    def test_world_position_beats_capture_pose(self):
        # Regression: a girder defect photographed from beside the west wall was
        # labelled canal_wall_W because assignment used the camera pose.
        from pipeline import correlation
        from pipeline.schema import Pose, make_detection
        d = make_detection("M", "drone", "rust_streak", 0.7, pose=Pose(x=0, y=0, z=0))
        d.world_pos = [10.0, 0.0, 5.0]
        emap = {"elements": [{"element_id": "canal_wall_W", "x": 0.5, "y": 0, "z": 0},
                             {"element_id": "bascule_girders", "x": 10, "y": 0, "z": 5}]}
        correlation.assign_locations([d], emap)
        self.assertEqual(d.location.structure.element_id, "bascule_girders")


class TestPerPlatformEvaluation(unittest.TestCase):
    def test_platforms_scored_separately_and_gt_sources_flagged(self):
        gts = [gt("a", pos=(0, 0, 1)),
               GTDefect(id="u", cls="scour", severity=3, element="toe", position=(0, 0, -3), platform="rov")]
        preds = [pred("p"),
                 Prediction(id="r", cls="scour", confidence=0.9, element="toe", position=(0, 0, -3),
                            platform="rov", source="ground-truth")]
        out = ev.evaluate(preds, gts)
        self.assertEqual(out["per_platform"]["drone"]["class_agnostic"]["tp"], 1)
        self.assertEqual(out["per_platform"]["rov"]["class_agnostic"]["tp"], 1)
        model, gt_src = ev.split_by_source(preds)
        self.assertEqual([p.id for p in model], ["p"])
        self.assertEqual([p.id for p in gt_src], ["r"])

    def test_loader_marks_underwater_gt_as_rov(self):
        with tempfile.TemporaryDirectory() as d:
            _write_rov_extract(Path(d))
            gts, _ = ev.load_ground_truth(Path(d) / "defects_gt.json")
        self.assertEqual({g.id: g.platform for g in gts},
                         {"U1": "rov", "U2": "rov", "A1": "drone", "FAR": "rov"})


COOP_SAMPLE = ROOT / "sample_data" / "run_20261024_134808"


@unittest.skipUnless(COOP_SAMPLE.exists(), "drone+ROV sample not present")
class TestCoopSample(unittest.TestCase):
    """The committed drone+ROV sample: 9 above-water + 4 underwater GT defects."""

    def test_rov_extract_loads(self):
        from pipeline import rov
        caps = rov.load_captures(str(COOP_SAMPLE))
        self.assertEqual(sum(1 for c in caps if c.kind == "sonar"), 1475)
        self.assertEqual(sum(1 for c in caps if c.kind == "rov_camera"), 688)

    def test_underwater_gt_attached_to_sonar(self):
        from pipeline import rov
        sonar = [c for c in rov.load_captures(str(COOP_SAMPLE)) if c.kind == "sonar"]
        dets = rov.gt_detections(str(COOP_SAMPLE), "M", sonar)
        self.assertEqual(sorted(getattr(d, "_gt_id") for d in dets), ["D10", "D11", "D12", "D13"])

    def test_rov_health_is_real(self):
        from pipeline import health
        h = health.build_health(str(COOP_SAMPLE))
        self.assertFalse(h["rov"]["is_filler"])
        self.assertEqual(h["rov"]["dives"], 4)          # D01, D03, D05, D07 coop tasks
        self.assertEqual(h["rov"]["nav"]["dvl_lock_pct"], 100.0)

    def test_sample_files_fit_in_git(self):
        big = [p for p in COOP_SAMPLE.rglob("*") if p.is_file() and p.stat().st_size > 50 * 1024 * 1024]
        self.assertEqual(big, [])


class TestSonarGeometry(unittest.TestCase):
    def test_raw_fan_pixels_to_polar(self):
        from pipeline import rov
        ax, ay = rov.FAN_APEX_PX
        # straight ahead, half range
        rng, brg = rov.box_to_polar([ax - 2, ay - rov.FAN_RADIUS_PX / 2 - 2,
                                     ax + 2, ay - rov.FAN_RADIUS_PX / 2 + 2], "raw", (300, 560))
        self.assertAlmostEqual(rng, rov.FAN_RANGE_M / 2, places=2)
        self.assertAlmostEqual(brg, 0.0, places=2)
        # measured left edge of the fan (col 16, row 168) sits at about -65 deg, full range
        rng, brg = rov.box_to_polar([15, 167, 17, 169], "raw", (300, 560))
        self.assertAlmostEqual(brg, -rov.FAN_HALF_DEG, delta=0.6)
        self.assertAlmostEqual(rng, rov.FAN_RANGE_M, delta=0.1)

    def test_unwarp_layout_to_polar(self):
        from pipeline import rov
        rows, cols = rov.UNWARP_SIZE
        rng, brg = rov.box_to_polar([cols - 3, 0, cols - 1, 2], "unwarp", (rows, cols))
        self.assertAlmostEqual(brg, rov.FAN_HALF_DEG, delta=0.5)   # right edge = starboard
        self.assertAlmostEqual(rng, rov.FAN_RANGE_M, delta=0.1)    # top row = far range

    def test_polar_to_world_uses_heading(self):
        from pipeline import rov
        import math as _m
        # heading north (yaw = pi/2), target 4 m dead ahead -> 4 m north
        self.assertEqual(rov.polar_to_world((1, 2, -3), _m.pi / 2, 4.0, 0.0), [1.0, 6.0, -3.0])
        # 90 deg to starboard while heading north -> east
        x, y, _ = rov.polar_to_world((0, 0, 0), _m.pi / 2, 4.0, 90.0)
        self.assertAlmostEqual(x, 4.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)

    def test_clustering_min_hits_and_rejections(self):
        import csv as _csv
        from pipeline import rov
        with tempfile.TemporaryDirectory() as d:
            bag = Path(d)
            _write_rov_extract(bag)
            rows = [  # two pings agree on one spot, one lone hit, one oversized box
                [100, 0, "shipwreck", 0.6, 1, 1, 9, 9, 3, 0, 5.0, 0.0, -2, "", "sonar-yolo@m"],
                [110, 1, "shipwreck", 0.4, 1, 1, 9, 9, 3, 0, 5.5, 0.3, -2, "", "sonar-yolo@m"],
                [110, 1, "mine_cylinder", 0.9, 1, 1, 9, 9, 3, 0, 40.0, 0.0, -2, "", "sonar-yolo@m"],
                [100, 0, "shipwreck", 0.95, 0, 0, 500, 290, 6, 0, 9.0, 0.0, -2,
                 "box covers 0.86 of frame", "sonar-yolo@m"],
            ]
            with (bag / "rov" / "sonar_detections_raw.csv").open("w", newline="") as fh:
                w = _csv.writer(fh)
                w.writerow(rov.CACHE_FIELDS)
                w.writerows(rows)
            dets = rov.sonar_model_detections(str(bag), "M", mode="raw")
            st = rov.sonar_cache_stats(str(bag), "raw")
        self.assertEqual(len(dets), 1)                       # lone hit + oversized box dropped
        self.assertEqual(dets[0].cls, "sonar_anomaly")
        self.assertEqual(dets[0].confidence, 0.6)
        self.assertEqual(getattr(dets[0], "_sonar_hits"), 2)
        self.assertEqual(dets[0].world_pos, [5.0, 0.0, -2.0])
        self.assertEqual(st["rejected"], {"too large": 1})


class TestMergeUsesWorldPosition(unittest.TestCase):
    def test_two_anomalies_from_one_ping_stay_separate(self):
        from pipeline import correlation
        from pipeline.schema import Pose, make_detection
        a = make_detection("M", "rov", "sonar_anomaly", 0.5, pose=Pose(x=0, y=0, z=-2))
        b = make_detection("M", "rov", "sonar_anomaly", 0.4, pose=Pose(x=0, y=0, z=-2))
        a.world_pos, b.world_pos = [4.0, 0.0, -2.0], [0.0, 6.0, -2.0]
        self.assertEqual(len(correlation.merge_duplicates([a, b])), 2)

    def test_same_spot_still_merges(self):
        from pipeline import correlation
        from pipeline.schema import Pose, make_detection
        a = make_detection("M", "rov", "sonar_anomaly", 0.5, pose=Pose(x=0, y=0, z=-2))
        b = make_detection("M", "rov", "sonar_anomaly", 0.4, pose=Pose(x=9, y=9, z=-2))
        a.world_pos, b.world_pos = [4.0, 0.0, -2.0], [4.2, 0.1, -2.0]
        self.assertEqual(len(correlation.merge_duplicates([a, b])), 1)


class TestSeverityExplanations(unittest.TestCase):
    """The dashboard shows *why* a severity was set; the trail must match the rule."""

    def _det(self, cls, conf, platform="drone", depth=None, change=None):
        from pipeline.schema import make_detection
        d = make_detection("M", platform, cls, conf)
        d.geometry.depth_mm = depth
        d.change_status = change
        return d

    def test_reason_names_the_rule_that_fired(self):
        from pipeline import severity
        state, why = severity.classify_with_reason(self._det("spall", 0.9, depth=60.0))
        self.assertEqual(state, "severe")
        self.assertIn("60.0 mm >= 50.0 mm severe threshold", why)
        state, why = severity.classify_with_reason(self._det("exposed_rebar", 0.9))
        self.assertEqual(state, "poor")
        self.assertIn("poor on presence", why)

    def test_low_confidence_cap_is_explained(self):
        from pipeline import severity
        state, why = severity.classify_with_reason(self._det("spall", 0.3, depth=60.0))
        self.assertEqual(state, "fair")
        self.assertIn("capped at fair", why)

    def test_classify_unchanged(self):
        from pipeline import severity
        for args in (("spall", 0.9, "drone", 60.0, None), ("pier_damage", 0.9, "rov", 35.0, "worsening"),
                     ("crack", 0.2, "drone", None, None)):
            d = self._det(*args)
            self.assertEqual(severity.classify(d), severity.classify_with_reason(d)[0])

    def test_run_severity_fills_reason_and_action_and_serialises(self):
        from pipeline import severity
        d = self._det("missing_block", 0.35, platform="rov")
        d.confirmed = False
        severity.run_severity([d])
        self.assertEqual(d.severity, "fair")             # poor on presence, capped by confidence
        self.assertTrue(d.severity_reason)
        self.assertIn("Monitor", d.recommended_action)
        self.assertIn("Tentative", d.recommended_action)
        self.assertIn("Sonar-only", d.recommended_action)
        out = d.to_dict()
        self.assertEqual(out["severity_reason"], d.severity_reason)
        self.assertEqual(out["recommended_action"], d.recommended_action)


class TestHardwareMock(unittest.TestCase):
    def test_deterministic_and_labelled(self):
        from pipeline import hardware
        a = hardware.simulate("run_X", 400, 400)
        self.assertEqual(a, hardware.simulate("run_X", 400, 400))
        self.assertNotEqual(a, hardware.simulate("run_Y", 400, 400))
        self.assertTrue(a["drone"]["is_simulated"] and a["rov"]["is_simulated"])
        self.assertIsNone(hardware.simulate("run_X", 400, 0, rov_flown=False)["rov"])

    def test_status_follows_thresholds(self):
        from pipeline import hardware as hw
        self.assertEqual(hw._status_hi(hw.MOTOR_TEMP_FAULT_C, hw.MOTOR_TEMP_WARN_C, hw.MOTOR_TEMP_FAULT_C), "fault")
        self.assertEqual(hw._status_hi(hw.MOTOR_TEMP_WARN_C, hw.MOTOR_TEMP_WARN_C, hw.MOTOR_TEMP_FAULT_C), "warn")
        self.assertEqual(hw._status_lo(10, hw.BATTERY_WARN_PCT, hw.BATTERY_FAULT_PCT), "fault")
        for p in ("drone", "rov"):
            h = hw.simulate("run_Z", 300, 300)[p]
            for m in h["motors"]:
                self.assertEqual(m["status"], hw._status_hi(m["temp_c"], hw.MOTOR_TEMP_WARN_C, hw.MOTOR_TEMP_FAULT_C))
            worst = max([c["status"] for c in h["components"]] + [m["status"] for m in h["motors"]],
                        key=hw.STATUS_RANK.get)
            self.assertEqual(h["overall"], worst)
            self.assertEqual(len(h["alerts"]),
                             sum(c["status"] != "ok" for c in h["components"]) + sum(m["status"] != "ok" for m in h["motors"]))

    def test_build_health_includes_hardware(self):
        from pipeline import health
        with tempfile.TemporaryDirectory() as d:
            h = health.build_health(d)
        self.assertIn("hardware", h)
        self.assertIsNone(h["hardware"]["rov"])          # no ROV extract -> not monitored


class TestKiroBackend(unittest.TestCase):
    def test_kiro_cli_call_and_fallback(self):
        import subprocess as sp
        from unittest import mock
        from pipeline import agents, config
        ok = sp.CompletedProcess([], 0, stdout="\x1b[32mElement X: fine [D-000000000001].\x1b[0m\n", stderr="")
        with mock.patch.object(config, "AGENT_BACKEND", "kiro"), \
             mock.patch.object(agents, "kiro_ready", return_value=True), \
             mock.patch("subprocess.run", return_value=ok) as run:
            out = agents._ask("sys", "user", 100)
        self.assertEqual(out, "Element X: fine [D-000000000001].")      # ANSI stripped
        args, kw = run.call_args
        self.assertEqual(args[0][1:3], ["chat", "--no-interactive"])
        self.assertFalse(any(a.startswith("--trust") for a in args[0]))  # no tools trusted
        self.assertIn("user", kw["input"])                               # prompt on stdin
        fail = sp.CompletedProcess([], 3, stdout="", stderr="auth required")
        with mock.patch.object(config, "AGENT_BACKEND", "kiro"), \
             mock.patch.object(agents, "kiro_ready", return_value=True), \
             mock.patch("subprocess.run", return_value=fail), mock.patch("time.sleep"):
            self.assertIsNone(agents._ask("sys", "user", 100))
        self.assertIn("auth required", agents.LAST_ERROR)


class TestReviewGate(unittest.TestCase):
    RES = {"mission_id": "M-1", "report": {"narrative": "Element A: crack [D-000000000001]."},
           "detections": [{"detection_id": "D-000000000001", "class": "crack", "severity": "fair",
                           "location": {"structure": {"element_id": "A"}}}]}

    def test_full_path_and_rules(self):
        from pipeline import review
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(review.load(d, self.RES)["state"], "draft")
            with self.assertRaises(review.ReviewError):
                review.apply(d, self.RES, "approve", reviewer="X")       # must submit first
            review.apply(d, self.RES, "submit")
            with self.assertRaises(review.ReviewError):
                review.apply(d, self.RES, "approve")                     # needs a name
            rec = review.apply(d, self.RES, "reject", reviewer="PE Smith", note="re-check D-01")
            self.assertEqual(rec["state"], "rejected")
            review.apply(d, self.RES, "resubmit")
            rec = review.apply(d, self.RES, "approve", reviewer="PE Smith")
            self.assertEqual(rec["state"], "approved")
            self.assertEqual([h["action"] for h in rec["history"]], ["submit", "reject", "resubmit", "approve"])
            self.assertEqual(review.load(d, self.RES)["state"], "approved")   # persisted

    def test_approval_does_not_survive_a_changed_report(self):
        import copy
        from pipeline import review
        with tempfile.TemporaryDirectory() as d:
            review.apply(d, self.RES, "submit")
            review.apply(d, self.RES, "approve", reviewer="PE Smith")
            changed = copy.deepcopy(self.RES)
            changed["detections"][0]["severity"] = "severe"
            rec = review.load(d, changed)
            self.assertEqual(rec["state"], "draft")
            self.assertIn("different report version", rec["note"])
            with self.assertRaises(review.ReviewError):                     # stale page
                review.apply(d, changed, "submit", expected_hash=review.report_hash(self.RES))
