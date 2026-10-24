"""Tests for the bridge-inspection route (sim/route.py).

Checks, with numpy only, that:
  * to_geo / ned_to_world are mutually consistent: a world point mapped to geo
    and back lands within 1 cm (using the same small-offset formulas);
  * every pier waypoint keeps >= 2.9 m horizontal clearance from that pier's
    column / cap-beam footprint;
  * every span waypoint stays >= 2.9 m from the girder soffit or the deck edge;
  * all mission altitudes relative to LAUNCH are > 1.0 m;
  * flight_plan() starts and ends over the LAUNCH pontoon.

route.py is imported by path so the test does not depend on sys.path layout.
"""
import math
import os
import importlib.util

import numpy as np
import pytest

# ---- import sim/route.py by path -------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROUTE_PATH = os.path.join(_HERE, "..", "sim", "route.py")
_spec = importlib.util.spec_from_file_location("route_under_test", _ROUTE_PATH)
route = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(route)

# Minimum required clearance (m). The route is designed to STANDOFF = 3.0 m, so
# a 2.9 m floor leaves a 0.1 m margin for rounding / curved footprints.
CLEAR = 2.9

# Bridge geometry that the route is designed around (mirrors sim/bridge_world.py).
COL_CY = (-4.6, 0.0, 4.6)      # the three columns per pier (y, metres)
COL_R = 0.75                   # column radius
CAP_HALF_X = 1.1               # cap beam half-width in x  (size 2.2 / 2)
CAP_HALF_Y = (route.W - 0.6) / 2.0   # cap beam half-width in y
CAP_H = 1.4
CAP_TOP = route.CAP_TOP
CAP_BOT = CAP_TOP - CAP_H


def _horizontal_dist_to_pier(px, pos):
    """Horizontal distance from a waypoint to the nearest pier solid at the
    waypoint's own height. Returns min over the three columns and the cap beam
    footprint (the cap only exists in its z-band)."""
    x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
    dists = []
    # columns: circles of radius COL_R centred at (px, cy), full height
    for cy in COL_CY:
        d_centre = math.hypot(x - px, y - cy)
        dists.append(d_centre - COL_R)
    # cap beam: axis-aligned box footprint, but only within its z-band.
    if CAP_BOT - 0.5 <= z <= CAP_TOP + 0.5:
        dx = max(abs(x - px) - CAP_HALF_X, 0.0)
        dy = max(abs(y - 0.0) - CAP_HALF_Y, 0.0)
        dists.append(math.hypot(dx, dy))
    return min(dists)


# ----------------------------------------------------------------------------
def test_to_geo_ned_world_roundtrip():
    """world -> geo -> world within 1 cm, using the route's own formulas."""
    rng = np.random.default_rng(0)
    for _ in range(200):
        world = np.array([rng.uniform(-60, 60),
                          rng.uniform(-60, 60),
                          rng.uniform(0, 12)])
        lat, lon, alt = route.to_geo(world)

        # Invert to_geo with the same small-offset (equirectangular) formulas.
        y = (lat - route.LAT0) * 111320.0
        x = (lon - route.LON0) * (111320.0 * math.cos(math.radians(route.LAT0)))
        z = alt + route.LAUNCH[2]
        back = np.array([x, y, z])

        assert np.linalg.norm(back - world) < 0.01, \
            f"roundtrip off by >1 cm: {world} -> {back}"


def test_ned_to_world_consistent_with_to_geo():
    """ned_to_world and to_geo agree on the horizontal frame: an NED offset
    from home, mapped to world then to geo, matches a direct lat/lon shift."""
    rng = np.random.default_rng(1)
    for _ in range(100):
        n, e, d = rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(-12, 0)
        world = route.ned_to_world(n, e, d)
        lat, lon, _ = route.to_geo(world)
        # north (n) must move latitude, east (e) must move longitude.
        exp_lat = route.LAT0 + (route.LAUNCH[1] + n) / 111320.0
        exp_lon = route.LON0 + (route.LAUNCH[0] + e) / (111320.0 * math.cos(math.radians(route.LAT0)))
        assert abs(lat - exp_lat) < 1e-9
        assert abs(lon - exp_lon) < 1e-9


def test_pier_waypoints_clearance():
    """Every pier waypoint keeps >= CLEAR m from its pier footprint."""
    route_wps = route.inspection_route()
    pier_wps = [(pos, el) for (pos, tgt, el) in route_wps if el.startswith("pier_")]
    assert pier_wps, "no pier waypoints found"
    for pos, el in pier_wps:
        k = int(el.split("_")[1]) - 1          # pier_1 -> index 0
        px = route.PIERS[k]
        d = _horizontal_dist_to_pier(px, pos)
        assert d >= CLEAR, (
            f"{el} waypoint {np.round(pos, 2)} only {d:.2f} m from pier "
            f"(need >= {CLEAR} m)")


def test_span_waypoints_clearance():
    """Every span waypoint stays >= CLEAR m from the girder soffit or deck edge."""
    route_wps = route.inspection_route()
    span_wps = [(pos, el) for (pos, tgt, el) in route_wps if el.startswith("span_")]
    assert span_wps, "no span waypoints found"
    deck_edge_y = route.W / 2.0
    soffit_z = route.G_BOT
    for pos, el in span_wps:
        x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
        if z < soffit_z - 1e-6:
            # underneath the deck: clearance is the vertical gap to the soffit.
            gap = soffit_z - z
            assert gap >= CLEAR, (
                f"{el} under-deck waypoint {np.round(pos, 2)} only {gap:.2f} m "
                f"below the girder soffit (need >= {CLEAR} m)")
        else:
            # alongside: clearance is the horizontal gap to the deck edge.
            gap = abs(y) - deck_edge_y
            assert gap >= CLEAR, (
                f"{el} alongside waypoint {np.round(pos, 2)} only {gap:.2f} m "
                f"from the deck edge (need >= {CLEAR} m)")


def test_mission_altitudes_above_launch():
    """All mission altitudes relative to LAUNCH are > 1.0 m."""
    plan = route.flight_plan()
    assert plan, "empty flight plan"
    for pos, tgt, el in plan:
        rel = float(pos[2]) - float(route.LAUNCH[2])
        assert rel > 1.0, (
            f"{el} waypoint {np.round(pos, 2)} is only {rel:.2f} m above LAUNCH "
            f"(need > 1.0 m)")


def test_flight_plan_starts_and_ends_over_launch():
    """flight_plan() begins and ends over the LAUNCH pontoon (x, y)."""
    plan = route.flight_plan()
    first_pos = np.asarray(plan[0][0])
    last_pos = np.asarray(plan[-1][0])
    launch_xy = np.asarray(route.LAUNCH[:2])
    assert np.linalg.norm(first_pos[:2] - launch_xy) < 1e-6, \
        f"plan does not start over LAUNCH: {first_pos}"
    assert np.linalg.norm(last_pos[:2] - launch_xy) < 1e-6, \
        f"plan does not end over LAUNCH: {last_pos}"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
