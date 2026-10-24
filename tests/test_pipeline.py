"""Smoke + behavior tests for the one-way pipeline. Standard library unittest,
runs with no third-party deps (mock mode). Run: python -m unittest -v
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pipeline import agents, correlation, severity  # noqa: E402
from pipeline.indexing import Capture, TimePoseIndex  # noqa: E402
from pipeline.orchestrator import run_pipeline  # noqa: E402
from pipeline.schema import Pose, make_detection  # noqa: E402

FIXTURE = str(Path(__file__).resolve().parent.parent / "fixtures" / "demo_mission.json")


class TestRetrace(unittest.TestCase):
    def setUp(self):
        self.idx = TimePoseIndex(max_dt_s=0.5, max_pose_m=2.0)
        self.idx.add(Capture("rov_camera", 100.0, "cam-a", Pose(x=0, y=0, z=0)))
        self.idx.add(Capture("rov_camera", 200.0, "cam-b", Pose(x=50, y=0, z=0)))

    def test_retrace_hits_within_gates(self):
        cap = self.idx.retrace("rov_camera", 100.1, Pose(x=0.5, y=0, z=0))
        self.assertIsNotNone(cap)
        self.assertEqual(cap.capture_id, "cam-a")

    def test_retrace_misses_outside_time_gate(self):
        self.assertIsNone(self.idx.retrace("rov_camera", 150.0, Pose(x=0, y=0, z=0)))

    def test_retrace_misses_outside_pose_gate(self):
        # time-close to cam-a but 10 m away -> turbid/off-target, no confirm
        self.assertIsNone(self.idx.retrace("rov_camera", 100.1, Pose(x=10, y=0, z=0)))


class TestSeverity(unittest.TestCase):
    def test_exposed_rebar_is_poor_on_presence(self):
        d = make_detection("M", "drone", "exposed_rebar", 0.9)
        self.assertEqual(severity.classify(d), "poor")

    def test_deep_spall_is_severe(self):
        d = make_detection("M", "drone", "spall", 0.9)
        d.geometry.depth_mm = 60.0
        self.assertEqual(severity.classify(d), "severe")

    def test_low_confidence_caps_at_fair(self):
        d = make_detection("M", "drone", "spall", 0.3)
        d.geometry.depth_mm = 60.0
        self.assertEqual(severity.classify(d), "fair")

    def test_worsening_bumps_one_step(self):
        d = make_detection("M", "rov", "pier_damage", 0.9)
        d.geometry.depth_mm = 35.0           # -> poor
        d.change_status = "worsening"
        self.assertEqual(severity.classify(d), "severe")


class TestReportValidation(unittest.TestCase):
    def test_unknown_id_claim_is_dropped(self):
        valid = {"D-000000000001"}
        text = ("Element A: spall [D-000000000001].\n"
                "Element B: crack 40 mm [D-ffffffffffff].")
        kept, dropped = agents._validate_report(text, valid)
        self.assertEqual(len(kept), 1)
        self.assertEqual(len(dropped), 1)
        self.assertIn("D-000000000001", kept[0])


class TestCorrelation(unittest.TestCase):
    def test_merge_collapses_duplicate_same_class(self):
        a = make_detection("M", "drone", "spall", 0.9, pose=Pose(x=0, y=0, z=0))
        b = make_detection("M", "drone", "spall", 0.7, pose=Pose(x=0.2, y=0, z=0))
        merged = correlation.merge_duplicates([a, b])
        self.assertEqual(len(merged), 1)
        self.assertEqual(merged[0].confidence, 0.9)

    def test_distinct_classes_not_merged(self):
        a = make_detection("M", "drone", "spall", 0.9, pose=Pose(x=0, y=0, z=0))
        b = make_detection("M", "drone", "crack", 0.9, pose=Pose(x=0, y=0, z=0))
        self.assertEqual(len(correlation.merge_duplicates([a, b])), 2)


class TestEndToEnd(unittest.TestCase):
    def test_full_mock_pipeline(self):
        result = run_pipeline(
            "M-TEST", FIXTURE,
            data_dir=str(Path(FIXTURE).parent),
            prior_dir=str(Path(FIXTURE).parent / "prior_cycle"),
        )
        self.assertEqual(result["metrics"]["defect_count"], 6)
        self.assertEqual(result["metrics"]["drone"], 3)
        self.assertEqual(result["metrics"]["rov"], 3)
        # PIER-02 pier_damage has no co-located rov_camera -> sonar-only
        self.assertEqual(result["metrics"]["sonar_only"], 1)
        # deterministic narrative always present and validated
        self.assertTrue(result["report"]["validated"])
        self.assertTrue(result["report"]["narrative"])
        # drone spall got LiDAR geometry from retrace
        spall = next(d for d in result["detections"]
                     if d["class"] == "spall")
        self.assertIsNotNone(spall["geometry"]["depth_mm"])


class TestRealBagHelpers(unittest.TestCase):
    def test_gt_severity_maps_directly(self):
        d = make_detection("M", "drone", "spall", 0.9)
        d._gt_severity = 3            # ground-truth rating wins over geometry
        d.geometry.depth_mm = 2.0     # tiny roughness must not downgrade it
        self.assertEqual(severity.classify(d), "severe")
        d._gt_severity = 1
        self.assertEqual(severity.classify(d), "fair")

    def test_surface_roughness_bounded_by_patch(self):
        from pipeline import lidar
        # A ~0.3 m flat patch with mm-scale noise -> small roughness, not metres.
        pts = []
        for i in range(10):
            for j in range(10):
                pts.append((i * 0.03, j * 0.03, 0.001 * ((i + j) % 3)))
        g = lidar._surface_roughness(pts)
        self.assertIsNotNone(g.depth_mm)
        self.assertLess(g.depth_mm, 50.0)   # mm-scale, never metres
        self.assertIsNone(g.area_mm2)       # no fabricated area

    def test_new_classes_validate(self):
        for cls in ("efflorescence", "rust_streak"):
            d = make_detection("M", "drone", cls, 0.6)  # must not raise
            self.assertEqual(d.cls, cls)


if __name__ == "__main__":
    unittest.main()
