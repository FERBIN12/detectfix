"""
Tests for bridge_triage.py.

Run with: .venv/bin/python -m pytest ai/test_bridge_triage.py -q

Covers:
  - NBI DDMMSSss coordinate parsing (lat/long, sign, edge cases)
  - risk-score monotonicity (worse inputs never lower the score)
  - underwater-inspection overdue logic (92B interval + 93B last date vs TODAY)
"""

from datetime import date

import pytest

import bridge_triage as bt


# --------------------------------------------------------------------------- #
# Coordinate parsing
# --------------------------------------------------------------------------- #

def test_parse_latitude_basic():
    # 41 deg 59' 35.40" N  (first row of MA22.txt)
    lat = bt.parse_coordinate("41593540", is_longitude=False)
    expected = 41 + 59 / 60 + 35.40 / 3600
    assert lat == pytest.approx(expected, abs=1e-6)


def test_parse_longitude_is_negative_west():
    # 071 deg 16' 57.60" W -> negative decimal degrees
    lon = bt.parse_coordinate("071165760", is_longitude=True)
    expected = -(71 + 16 / 60 + 57.60 / 3600)
    assert lon == pytest.approx(expected, abs=1e-6)
    assert lon < 0


def test_parse_longitude_three_degree_digits():
    # Longitude uses 3 degree digits; make sure we don't misalign minutes/sec.
    lon = bt.parse_coordinate("069565210", is_longitude=True)
    expected = -(69 + 56 / 60 + 52.10 / 3600)
    assert lon == pytest.approx(expected, abs=1e-6)


def test_parse_coordinate_zero_and_blank_are_none():
    assert bt.parse_coordinate("00000000", is_longitude=False) is None
    assert bt.parse_coordinate("", is_longitude=False) is None
    assert bt.parse_coordinate(None, is_longitude=True) is None
    assert bt.parse_coordinate("abc", is_longitude=False) is None


def test_massachusetts_coordinates_in_expected_box():
    # MA roughly lat 41-43 N, lon -73 to -70. Sanity-check a real row.
    lat = bt.parse_coordinate("42072675", is_longitude=False)
    lon = bt.parse_coordinate("070564706", is_longitude=True)
    assert 41 <= lat <= 43
    assert -74 <= lon <= -69


# --------------------------------------------------------------------------- #
# Interval / date parsing
# --------------------------------------------------------------------------- #

def test_underwater_interval_parsing():
    assert bt.parse_underwater_interval_months("Y36") == 36
    assert bt.parse_underwater_interval_months("Y12") == 12
    assert bt.parse_underwater_interval_months("N  ") is None
    assert bt.parse_underwater_interval_months("") is None
    # Required but no interval -> default 36.
    assert bt.parse_underwater_interval_months("Y") == 36


def test_last_underwater_date_mmyy():
    assert bt.parse_last_underwater_date("0819") == date(2019, 8, 1)
    assert bt.parse_last_underwater_date("1221") == date(2021, 12, 1)
    assert bt.parse_last_underwater_date("") is None
    assert bt.parse_last_underwater_date(None) is None
    assert bt.parse_last_underwater_date("1399") is None  # invalid month


# --------------------------------------------------------------------------- #
# Overdue logic
# --------------------------------------------------------------------------- #

def _row(**kw):
    """Build a minimal row dict with sane defaults for scoring helpers."""
    base = {
        "DECK_COND_058": "7",
        "SUPERSTRUCTURE_COND_059": "7",
        "SUBSTRUCTURE_COND_060": "7",
        "SCOUR_CRITICAL_113": "N",
        "YEAR_BUILT_027": "2000",
        "ADT_029": "1000",
        "UNDWATER_LOOK_SEE_092B": "N",
        "UNDWATER_LAST_DATE_093B": "",
        "SERVICE_UND_042B": "1",
        "BRIDGE_CONDITION": "G",
        "STRUCTURE_NUMBER_008": "TEST",
        "FEATURES_DESC_006A": "RIVER",
        "FACILITY_CARRIED_007": "ROAD",
        "LAT_016": "42000000",
        "LONG_017": "071000000",
    }
    base.update(kw)
    return base


def test_overdue_not_required_when_no_underwater():
    sub, info = bt._overdue_subscore(_row(UNDWATER_LOOK_SEE_092B="N"))
    assert sub == 0.0
    assert info["required"] is False
    assert info["overdue"] is False


def test_overdue_on_schedule():
    # Last inspected recently relative to TODAY (2026-10), 36-month interval.
    row = _row(UNDWATER_LOOK_SEE_092B="Y36", UNDWATER_LAST_DATE_093B="0625")  # Jun 2025
    sub, info = bt._overdue_subscore(row)
    assert info["required"] is True
    assert info["overdue"] is False
    assert sub == 0.0
    assert info["due_date"] == "2028-06-01"


def test_overdue_past_due():
    # Last inspected long ago -> overdue.
    row = _row(UNDWATER_LOOK_SEE_092B="Y36", UNDWATER_LAST_DATE_093B="0115")  # Jan 2015
    sub, info = bt._overdue_subscore(row)
    assert info["required"] is True
    assert info["overdue"] is True
    assert info["months_overdue"] > 0
    assert sub > 0.0


def test_overdue_missing_last_date_treated_overdue():
    row = _row(UNDWATER_LOOK_SEE_092B="Y36", UNDWATER_LAST_DATE_093B="")
    sub, info = bt._overdue_subscore(row)
    assert info["required"] is True
    assert info["overdue"] is True
    assert sub == pytest.approx(0.8)


# --------------------------------------------------------------------------- #
# Scoring monotonicity
# --------------------------------------------------------------------------- #

def test_worse_condition_increases_score():
    good = bt.score_row(_row(DECK_COND_058="8", SUPERSTRUCTURE_COND_059="8",
                             SUBSTRUCTURE_COND_060="8"))
    bad = bt.score_row(_row(DECK_COND_058="2", SUPERSTRUCTURE_COND_059="8",
                            SUBSTRUCTURE_COND_060="8"))
    assert bad["score"] > good["score"]


def test_scour_critical_increases_score():
    ok = bt.score_row(_row(SCOUR_CRITICAL_113="8"))
    crit = bt.score_row(_row(SCOUR_CRITICAL_113="2"))
    assert crit["score"] > ok["score"]


def test_lower_scour_code_is_higher_or_equal_risk():
    # Monotonic within critical range: 0 >= 1 >= 2 >= 3 in risk.
    scores = [bt.score_row(_row(SCOUR_CRITICAL_113=str(c)))["score"] for c in range(0, 4)]
    assert scores == sorted(scores, reverse=True)


def test_older_bridge_increases_score():
    new = bt.score_row(_row(YEAR_BUILT_027="2020"))
    old = bt.score_row(_row(YEAR_BUILT_027="1920"))
    assert old["score"] > new["score"]


def test_higher_traffic_increases_score():
    low = bt.score_row(_row(ADT_029="100"))
    high = bt.score_row(_row(ADT_029="100000"))
    assert high["score"] > low["score"]


def test_overdue_increases_score():
    on_time = bt.score_row(_row(UNDWATER_LOOK_SEE_092B="Y36",
                                UNDWATER_LAST_DATE_093B="0625"))
    overdue = bt.score_row(_row(UNDWATER_LOOK_SEE_092B="Y36",
                                UNDWATER_LAST_DATE_093B="0110"))
    assert overdue["score"] > on_time["score"]


def test_all_worse_dominates():
    # A bridge that is worse on every axis must out-score a pristine one.
    best = bt.score_row(_row(DECK_COND_058="9", SUPERSTRUCTURE_COND_059="9",
                             SUBSTRUCTURE_COND_060="9", SCOUR_CRITICAL_113="9",
                             YEAR_BUILT_027="2021", ADT_029="10",
                             UNDWATER_LOOK_SEE_092B="N"))
    worst = bt.score_row(_row(DECK_COND_058="0", SUPERSTRUCTURE_COND_059="0",
                              SUBSTRUCTURE_COND_060="0", SCOUR_CRITICAL_113="0",
                              YEAR_BUILT_027="1900", ADT_029="150000",
                              UNDWATER_LOOK_SEE_092B="Y36",
                              UNDWATER_LAST_DATE_093B="0100"))
    assert worst["score"] > best["score"]
    assert 0 <= best["score"] <= 100
    assert 0 <= worst["score"] <= 100


def test_score_bounds_and_reasons():
    r = bt.score_row(_row(SCOUR_CRITICAL_113="1", DECK_COND_058="3"))
    assert 0 <= r["score"] <= 100
    assert isinstance(r["reasons"], list)
    assert 1 <= len(r["reasons"]) <= 3


# --------------------------------------------------------------------------- #
# SNBI mapping
# --------------------------------------------------------------------------- #

def test_snbi_fields_structure_and_no_invented_ids():
    m = bt.snbi_fields(_row(SCOUR_CRITICAL_113="2"))
    assert "SCOUR_CRITICAL_113" in m
    for key, field in m.items():
        assert "mapping_note" in field and field["mapping_note"]
        assert "snbi_item" in field  # may be None where uncertain
    # Legacy value is carried through.
    assert m["SCOUR_CRITICAL_113"]["nbi_value"] == "2"
