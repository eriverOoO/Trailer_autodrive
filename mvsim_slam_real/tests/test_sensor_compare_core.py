"""Check missing beams, scan wrap, and timestamp selection for field captures."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from sensor_compare_core import nearest_scan, scan_groups


def scan(ranges, increment=.01, stamp=10.):
    return SimpleNamespace(ranges=ranges, range_min=.1, range_max=12.,
                           angle_min=0., angle_increment=increment,
                           header=SimpleNamespace(stamp=SimpleNamespace(
                               sec=int(stamp), nanosec=round((stamp-int(stamp))*1e9))))


def test_missing_beams_do_not_bridge_distinct_groups():
    groups = scan_groups(scan([1.,1.,1.,float('inf'),1.,1.,1.,float('nan'),0.,15.]))
    assert [len(g) for g in groups] == [3,3]


def test_distance_discontinuity_and_small_specks():
    groups = scan_groups(scan([1.,1.,1.,3.,3.,3.,float('inf'),1.,1.]))
    assert [len(g) for g in groups] == [3,3]
    np.testing.assert_allclose(np.linalg.norm(groups[1],axis=1), 3.)


def test_full_circle_endpoints_merge_but_partial_scan_does_not():
    values = [1.,1.]+[float('inf')]*356+[1.,1.]
    assert [len(g) for g in scan_groups(scan(values,2*math.pi/360))] == [4]
    assert scan_groups(scan(values,math.pi/360)) == []


def test_only_timestamp_near_real_scans_are_selected():
    a,b = scan([],stamp=10.),scan([],stamp=10.15)
    result,delta = nearest_scan([a,b],10_130_000_000)
    assert result is b
    assert abs(delta-.02)<1e-8
    assert nearest_scan([a,b],11_000_000_000)[0] is None
    assert nearest_scan([],10_000_000_000) == (None,None)
    assert nearest_scan([a],0) == (None,None)
    assert nearest_scan([scan([],stamp=0)],1)[0] is None
