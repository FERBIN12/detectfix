"""Inspection route + world <-> geo frame shared by the autopilot bridge (mav_bridge.py) and the Isaac world.

World frame: metres, X = east, Y = north, Z = up, origin = bridge centre at the water line. The origin sits on the
North Washington Street (Charlestown) Bridge over the Charles River by TD Garden: NBI 2022 record B1601637HMUNNBI,
built 1900, 5 spans / 129 m, ~43,300 vehicles a day, scour critical (item 113 = 3) and Poor. (Since replaced by the
Bill Russell Bridge.)
"""
import math
import numpy as np

LAT0, LON0 = 42.367800, -71.059140            # NBI items 16/17, N Washington St (Charlestown) Bridge over the Charles, by TD Garden
WATER_AMSL = 2.0                             # assumed river surface elevation (m)
LAUNCH = np.array([20.0, -40.0, 0.4])         # launch pontoon on the river, downstream of the bridge (SITL home)

PIERS = [-38.7, -12.9, 12.9, 38.7]
W = 15.0
DECK_TOP, DECK_T, GIRDER_D = 10.5, 0.25, 1.6
G_BOT = DECK_TOP - DECK_T - GIRDER_D          # girder soffit (8.65 m)
CAP_TOP = G_BOT
SUPPORTS = [-64.5, *PIERS, 64.5]
STANDOFF = 3.0


def to_geo(p):
    """world (x east, y north, z up) -> (lat, lon, altitude relative to the SITL home on the pontoon)."""
    lat = LAT0 + p[1] / 111320.0
    lon = LON0 + p[0] / (111320.0 * math.cos(math.radians(LAT0)))
    return lat, lon, float(p[2] - LAUNCH[2])


def ned_to_world(n, e, d):
    """ArduPilot LOCAL_POSITION_NED (origin = home) -> world."""
    return LAUNCH + np.array([e, n, -d])


def inspection_route():
    """[(position, look-at target, element)] keeping STANDOFF metres off every inspected face."""
    wp = []
    for k in (2, 1):                                                   # the two river piers nearest the launch
        px = PIERS[k]
        for z in (1.8, 5.0, CAP_TOP - 0.8):
            ry = 4.6 + 0.75 + STANDOFF if z < CAP_TOP - 1.5 else W / 2 - 0.3 + STANDOFF
            for a in np.linspace(-math.pi / 2, 1.5 * math.pi, 18, endpoint=False):
                pos = np.array([px + (1.1 + STANDOFF) * math.cos(a), ry * math.sin(a), z])
                wp.append((pos, np.array([px, float(np.clip(pos[1], -4.6, 4.6)), z]), f"pier_{k + 1}"))
    for b in (3, 2, 1):                                                # main spans: east to west alongside, back underneath
        x0, x1 = SUPPORTS[b] + 2, SUPPORTS[b + 1] - 2
        for x in np.linspace(x1, x0, 6):
            wp.append((np.array([x, -W / 2 - STANDOFF, G_BOT + 0.5]), np.array([x, -W / 2 + 1.0, G_BOT + 0.6]), f"span_{b + 1}"))
        for x in np.linspace(x0, x1, 6):
            wp.append((np.array([x, 0.0, G_BOT - STANDOFF]), np.array([x, 0.5, G_BOT]), f"span_{b + 1}"))
    return wp


def flight_plan():
    """Full autopilot mission: climb off the pontoon, approach, inspect, return over the pontoon (then RTL lands)."""
    route = inspection_route()
    approach = [(np.array([LAUNCH[0], LAUNCH[1], 6.0]), np.array([0, 0, 5.0]), "transit"),
                (np.array([17.0, -14.0, 3.0]), np.array([12.9, 0, 3.0]), "transit")]
    back = [(np.array([-30.0, -24.0, 6.0]), np.array([0, 0, 6.0]), "transit"),
            (np.array([LAUNCH[0], LAUNCH[1], 6.0]), np.array([LAUNCH[0], LAUNCH[1], 0.0]), "transit")]
    return approach + route + back
