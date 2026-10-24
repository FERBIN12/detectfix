#!/usr/bin/env python3
"""Micro-benchmark for the render-loop-critical paths in sim/.

Measures per-call wall time for:

  * ``RosPub.map(Pw)`` with a fresh 400,000-point world buffer over a 50 m cube
    (the worst case: full voxel fold + pack + publish);
  * ``RosPub.map(Pw)`` incremental: the same buffer grown by +5,000 new points
    (the steady-state render-loop case -- only the new points are processed);
  * ``thermal.synth(labels, id2class, depth, t)`` on a 540x675 scene.

Targets (this machine):  map <= 30 ms,  synth <= 12 ms.

Run:
    source /opt/ros/jazzy/setup.bash && \
        ~/IsaacLab/env_isaaclab/bin/python scripts/bench_ros.py
"""
from __future__ import annotations

import importlib.util
import os
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.join(_HERE, "..")


def _load(mod_name: str, rel_path: str):
    path = os.path.join(_ROOT, rel_path)
    spec = importlib.util.spec_from_file_location(mod_name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _fmt(name: str, ms: float, target: float) -> str:
    ok = "OK " if ms <= target else "FAIL"
    return f"[{ok}] {name:<28} {ms:8.2f} ms   (target <= {target:.0f} ms)"


def bench_map(reps: int = 5) -> tuple[float, float]:
    """Return (fresh_400k_ms, incremental_5k_ms), best-of-reps per call."""
    ros_pub = _load("ros_pub_bench", os.path.join("sim", "ros_pub.py"))
    import rclpy

    if not rclpy.ok():
        rclpy.init()
    pub = ros_pub.RosPub(node_name="bench_inspection_drone")

    rng = np.random.default_rng(0)
    # 400k random points over a 50 m cube (matches the reported measurement).
    base = (rng.random((400_000, 3), dtype=np.float64) * 50.0).astype(np.float32)

    # --- fresh 400k: time a cold map() (reset incremental + rate-limit state) -
    fresh_times = []
    for _ in range(reps):
        pub._reset_map_state()
        pub._last_map_t = -1.0
        t0 = time.perf_counter()
        pub.map(base)
        fresh_times.append((time.perf_counter() - t0) * 1e3)

    # --- incremental +5k: buffer already folded in, append 5k new points ------
    incr_times = []
    for _ in range(reps):
        pub._reset_map_state()
        pub._last_map_t = -1.0
        pub.map(base)                       # prime: whole buffer seen
        extra = (rng.random((5_000, 3), dtype=np.float64) * 50.0).astype(np.float32)
        grown = np.vstack([base, extra])
        pub._last_map_t = -1.0              # allow the next publish
        t0 = time.perf_counter()
        pub.map(grown)
        incr_times.append((time.perf_counter() - t0) * 1e3)

    pub.close()
    if rclpy.ok():
        rclpy.shutdown()
    return min(fresh_times), min(incr_times)


def bench_synth(reps: int = 10) -> float:
    """Return best-of-reps per-call time for thermal.synth (ms)."""
    thermal = _load("thermal_bench", os.path.join("sim", "thermal.py"))

    h, w = 540, 675
    labels = np.zeros((h, w), dtype=np.int32)
    labels[:, : w // 2] = 1                 # {1: "granite"} like the measurement
    id2class = {1: "granite"}
    depth = np.full((h, w), 5.0, dtype=np.float32)
    rng = np.random.default_rng(0)

    # warm up (first call fills the cached noise field).
    thermal.synth(labels, id2class, depth, t=0.0, rng=rng)

    times = []
    for i in range(reps):
        t0 = time.perf_counter()
        thermal.synth(labels, id2class, depth, t=float(i), rng=rng)
        times.append((time.perf_counter() - t0) * 1e3)
    return min(times)


def main() -> int:
    fresh_ms, incr_ms = bench_map()
    synth_ms = bench_synth()

    print("=== sim render-loop micro-benchmark ===")
    print(_fmt("map  fresh 400k", fresh_ms, 30.0))
    print(_fmt("map  incremental +5k", incr_ms, 30.0))
    print(_fmt("thermal.synth 540x675", synth_ms, 12.0))

    ok = fresh_ms <= 30.0 and incr_ms <= 30.0 and synth_ms <= 12.0
    print("\nALL TARGETS MET" if ok else "\nTARGETS NOT MET")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
