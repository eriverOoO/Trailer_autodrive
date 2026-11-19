"""Validate replay inputs before any publishers are created."""
import json
from pathlib import Path
import sys

import pytest

W=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(W/'tools'))
from replay_sensor_capture import read_records


def test_saved_capture_keeps_all_samples_and_orders_header_times():
    directory=W/'artifacts/sensor_compare_push/20261118_155446'
    if not (directory/'measurements.jsonl').is_file():
        pytest.skip('Recorded field capture not available')
    rows=read_records(directory)
    assert sum(row['type']=='image' for row in rows)==43
    assert sum(row['type']=='scan' for row in rows)==232
    assert all(a['stamp_ns']<=b['stamp_ns'] for a,b in zip(rows,rows[1:]))


def test_replay_rejects_image_paths_outside_capture(tmp_path):
    outside=tmp_path/'outside.jpg';outside.write_bytes(b'not published')
    capture=tmp_path/'capture';capture.mkdir()
    (capture/'measurements.jsonl').write_text('\n'.join(json.dumps(row) for row in [
        {'type':'image','stamp_ns':1,'file':'../outside.jpg'}, {'type':'scan','stamp_ns':2}]))
    with pytest.raises(ValueError,match='outside capture'):
        read_records(capture)
