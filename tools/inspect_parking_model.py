"""Print measured SDF geometry and initial visual calibration as JSON."""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/trailer_parking_pkg'))
from trailer_parking_pkg.calibration import simulation_calibration


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-index', type=int, default=1)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    config = simulation_calibration(ROOT/'src/simulation_pkg/models/prius_hybrid/model.sdf',
        ROOT/'src/simulation_pkg/simulation_pkg/lib/load_towing_system_node.py', args.start_index)
    config['parked_vehicle_center_distance'] = math.hypot(7.163876999707-7.151418196919, 7.919384043882+7.725951808349)
    config['approx_clear_gap_length'] = config['parked_vehicle_center_distance']-config['body_length']
    config['painted_slot_width'] = None  # No dimensioned slot object in track.world.
    text = json.dumps(config, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text+'\n', encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
