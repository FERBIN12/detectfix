"""Craigie Drawbridge + Charles River Dam lock canal (Route 28 by the Museum of Science, Boston): geometry, geo frame, route.

From the NBI 2022 record B160134EMDOTNBI (Craigie drawbridge, steel bascule, 19.2 m main span, 25.7 m deck, built 1910,
rebuilt 2011, 43,250 vehicles/day) and Wikimedia Commons photos (ref/): the bridge crosses the old lock canal of the 1910
dam; granite-block canal walls; two Beaux-Arts granite towers flank the canal at the bridge; the Lechmere Viaduct (Green
Line, green steel arch) runs right behind it; clearance below 4.6 m. The neighbouring 10-span dam bridge (B160134ET) is
rated Poor (substructure 4, channel 4).
World frame: metres, X = east, Y = north, Z = up, origin = centre of the drawbridge at the water line.
Canal frame: c along the canal (positive = downstream, toward the Zakim), r across it; the road runs along r.
"""
import math
import numpy as np

LAT0, LON0 = 42.367153, -71.069106                     # NBI items 16/17 of the Craigie drawbridge (B160134EM)
_K = 111320.0 * math.cos(math.radians(LAT0))
_ET = (42.368614, -71.071864)                           # the 10-span dam bridge, along Route 28 to the north-west


def geo_to_world(lat, lon):
    return np.array([(lon - LON0) * _K, (lat - LAT0) * 111320.0, 0.0])


_road = geo_to_world(*_ET)[:2]; R = _road / np.linalg.norm(_road)    # road direction (north-west)
C = np.array([-R[1], R[0]])                                          # canal direction, 90 deg ccw from the road (north-east, downstream)
if C[0] < 0: C = -C
HEADING_C = math.degrees(math.atan2(C[1], C[0]))                     # canal axis angle in the world XY plane


def cw(c, r, z=0.0):
    p = C * c + R * r
    return np.array([p[0], p[1], z])


CANAL_W = 13.0                       # water width between the granite walls
WALL_TOP = 3.6                       # wall coping above the water
DECK_W = 25.7                        # bridge deck width along the canal (NBI item 52)
SPAN = 19.2                          # bascule span across the canal (NBI item 48)
CLEAR = 4.6                          # clearance below the bridge
DECK_TOP = CLEAR + 1.3
CANAL_C0, CANAL_C1 = -90.0, 45.0     # canal extent (upstream lock approach .. downstream)
LAUNCH = cw(-62.0, 0.0, 0.62)        # landing pad on the inspection RIB in the canal, upstream of the bridge (SITL home)
WATER_AMSL = 2.0
STANDOFF = 2.5


def to_geo(p):
    return LAT0 + p[1] / 111320.0, LON0 + p[0] / _K, float(p[2] - LAUNCH[2])


def ned_to_world(n, e, d):
    return LAUNCH + np.array([e, n, -d])


def under_deck(p):
    c, r = float(np.dot(p[:2], C)), float(np.dot(p[:2], R))
    return abs(c) < DECK_W / 2 + 0.5 and abs(r) < SPAN / 2 + 1 and p[2] < DECK_TOP


def inspection_route():
    """Down the canal along the east wall (two heights), under the drawbridge (girders + bearings), back along the west wall."""
    wp = []
    rE, rW = CANAL_W / 2 - STANDOFF, -(CANAL_W / 2 - STANDOFF)
    for c in np.linspace(-50, -16, 6):
        wp.append((cw(c, rE, 1.4), cw(c, CANAL_W / 2, 1.2), "canal_wall_E"))
    for c in np.linspace(-16, 16, 9):                                              # under the bascule: gimbal up at the girders
        wp.append((cw(c, rE * 0.5, 2.6), cw(c, rE * 0.5 + 1.0, DECK_TOP - 1.2), "bascule_girders"))
    for c in np.linspace(16, 30, 3):
        wp.append((cw(c, rE, 2.4), cw(c, CANAL_W / 2, 2.2), "canal_wall_E"))
    for c in np.linspace(30, -16, 8):
        wp.append((cw(c, rW, 2.4), cw(c, -CANAL_W / 2, 2.0), "canal_wall_W"))
    for c in np.linspace(-16, -50, 6):
        wp.append((cw(c, rW, 1.4), cw(c, -CANAL_W / 2, 1.2), "canal_wall_W"))
    return wp


def flight_plan():
    up = [(LAUNCH + np.array([0, 0, 2.5]), cw(0, 0, 3.0), "transit"), (cw(-52, rE_ := CANAL_W / 2 - STANDOFF, 2.0), cw(-52, CANAL_W / 2, 1.5), "transit")]
    back = [(cw(-55, 0, 2.5), cw(-70, 0, 1.0), "transit"), (LAUNCH + np.array([0, 0, 2.5]), LAUNCH, "transit")]
    return up + inspection_route() + back
