"""Unpublished boundary cases; hardware verification is recorded separately."""
import pytest
from tractor_trailer_rviz_2d.timestamp_guard import SensorTimestampGuard


def test_observed_rollover_and_missing_packet():
    g=SensorTimestampGuard()
    g.observe(59992.,1.)
    g.observe(2.,1.01)
    g.observe(32.,1.04)
    assert g.rollovers==1 and g.missing_packets==2
    assert g.event=='PACKET_GAP'


@pytest.mark.parametrize('first,last,dt',[
    (12342.,2.,.01),       # reset outside rollover boundary
    (12342.,12332.,.01),   # reordered old packet
    (12342.,12342.,.01),   # duplicate packet
    (59992.,2.,.6),        # long gap at a plausible boundary
    (59992.,500.,.01),     # implausible boundary step
    (2.,59992.,.01),       # previous-cycle packet after rollover
    (12000.,13000.,.01),   # clock advance inconsistent with host time
])
def test_faults_do_not_advance_epoch(first,last,dt):
    g=SensorTimestampGuard();g.observe(first,1.)
    with pytest.raises(ValueError):g.observe(last,1.+dt)
    assert g.previous==first and g.rollovers==0
