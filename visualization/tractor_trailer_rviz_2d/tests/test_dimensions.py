"""Shared dimensions reach the actual RViz markers in both display modes."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.time import Time
import yaml

from tractor_trailer_rviz_2d.dimensions import default_geometry_file, sized_polygons
from tractor_trailer_rviz_2d.render_adapter import Renderer


def test_shared_dimensions_and_config_changes_reach_markers(tmp_path):
    root = Path(__file__).resolve().parents[1]
    outlines = json.loads((root / 'models/reference_footprints.json').read_text())['polygons']
    source = default_geometry_file()
    original = source.read_text()
    geometry = yaml.safe_load(source.read_text())
    for changed in (False, True):
        if changed:
            geometry['tractor_length'] = 3.0
            geometry['trailer_width'] = 0.8
        config = tmp_path / 'geometry.yaml'
        config.write_text(yaml.safe_dump(geometry))
        polygons = sized_polygons(outlines, config)
        node = SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=10**9)))
        renderer = Renderer(node, polygons, [-0.545, 0.0])
        batches = [renderer.render([2., -1., 0., 0.], 'TEST', 'map'),
                   renderer.render_tractor([2., -1.], 0., 'TEST', 'map')]
        for batch in batches:
            bodies = [m for m in batch.markers if m.ns == 'current_body']
            for marker in bodies:
                role = ('tractor', 'trailer')[marker.id]
                span = np.ptp([[p.x, p.y] for p in marker.points], axis=0)
                np.testing.assert_allclose(span, [geometry[role + '_length'], geometry[role + '_width']])
    # The hypothetical 3 m edit was made only in the temporary test file.
    assert source.read_text() == original


@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf')])
def test_invalid_dimensions_are_rejected(tmp_path, value):
    geometry = dict(tractor_length=value, tractor_width=.585, trailer_length=.97, trailer_width=.585)
    config = tmp_path / 'invalid.yaml'
    config.write_text(yaml.safe_dump(geometry))
    with pytest.raises(ValueError, match='finite and positive'):
        sized_polygons([[[0, 0], [1, 0], [1, 1]]] * 2, config)
